#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <opencv2/highgui/highgui.hpp>
 #include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/tracking.hpp>




#include "utility/utility.h"


#include "estimator.h"
#include "tracker.h"

#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;
bbTracker_t bbTracker_;
visualization_msgs::MarkerArray cube_list, line_list;
ros::Publisher marker_pub, track_image_pub_, cube_pub, optical_image_pub_ ;
ros::Publisher pub_corner_point;
Utility::all_locked_box locked_box_vec;
double yolo_img_time, yolo_bbox_time, img_time;
//image_transport::Publisher optical_image_pub_;
cv_bridge::CvImageConstPtr out_img, display_all,optical_result_bridge;
cv::Mat optical_img;

///Parameters
 float thresh_IOU=0.4;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf; //collection of 3D point
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

//args: IMU , out: tmp_P, tmp_V, acc_0, gyr_0 (linear accel, angular velocity)
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

//propagate all IMU measurements contained in imu_buf
void update()
{

    //take the last values of the state estimator
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE]; //position of IMU in G
    tmp_Q = estimator.Rs[WINDOW_SIZE]; //orienttion of IMU in G
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front()); //propagate the IMU measurement

}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{

    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}


void publish_rviz(const ros::Time& publish_time){


    bbTracker_.lock_bbox(); // check if we can lock any bbox

    geometry_msgs::Point ray0;
            ray0.x = tmp_P[0];
            ray0.y = tmp_P[1];
            ray0.z = tmp_P[2];
//if (bb_sub_.getNumPublishers()>0 ){      //for (int i=0; i<ray_State_vect.size();i++){

   if (bbTracker_.bbox_State_vect.size()>0 ){
       line_list.markers.resize(bbTracker_.bbox_State_vect.size());

      // line_list.markers[i].points=[];
bool new_vec=false;

   for (unsigned i =0; i<bbTracker_.bbox_State_vect.size(); i++){ // for each state
cout << "locked proba" << bbTracker_.bbox_State_vect[i].lock_proba << endl;
//we transfer the locked vectors to the global vec
       //for (int p=0; p<bbTracker_.bbox_State_vect[i].locked_vec.size(); p++){ //for all the new locked vector of this state
if (bbTracker_.bbox_State_vect[i].lock_proba >= 15){
           if (locked_box_vec.bbox_id_vec.size()>0){
            auto it = find(locked_box_vec.bbox_id_vec.begin(), locked_box_vec.bbox_id_vec.end(), (bbTracker_.bbox_State_vect[i].bbox_id));
            new_vec=(it == locked_box_vec.bbox_id_vec.end());
           }else if (locked_box_vec.bbox_id_vec.size()==0) {
               new_vec=true;

           }
           if  (new_vec && bbTracker_.bbox_State_vect[i].nb_detected>5 ) { //if we never locked this bbox state
           Utility::locked_box bbox;
           bbox.bbox_id = bbTracker_.bbox_State_vect[i].bbox_id;
           bbox.center = bbTracker_.bbox_State_vect[i].locked_bbox.second;
           cout << "Add locked box in rviz publisher with center" <<  bbox.center.transpose() << " and locked size is "<< locked_box_vec.bbox_id_vec.size()<< endl;
            //add width lenght and corners
           locked_box_vec.bbox_id_vec.push_back(bbTracker_.bbox_State_vect[i].bbox_id);
            locked_box_vec.locked_boxes.push_back(bbox);
          // bbTracker_.bbox_State_vect[i].locked_vec.erase(bbTracker_.bbox_State_vect[i].locked_vec.begin() + p);
           cout << "Add locked bbox with" <<  bbox.bbox_id <<  endl;

      } else if (locked_box_vec.bbox_id_vec.size()>0) { //we update this locked box (if 4 corner instead of three?
           Utility::locked_box bbox;
           bbox.bbox_id = bbTracker_.bbox_State_vect[i].bbox_id;
           auto it = find(locked_box_vec.bbox_id_vec.begin(), locked_box_vec.bbox_id_vec.end(), (bbTracker_.bbox_State_vect[i].bbox_id));

           //cout <<"CHECK if index is fitting. For "<<bbTracker_.bbox_State_vect[i].bbox_id << "is situated in " << index << "position of this vector" <<locked_box_vec.bbox_id_vec << endl;
           int index = std::distance( locked_box_vec.bbox_id_vec.begin(), it ); //index is the index of the locked box with this id

           bbox.center = bbTracker_.bbox_State_vect[i].locked_bbox.second;
           locked_box_vec.locked_boxes[index] = bbox;
           locked_box_vec.count =0;
           cout << "Update locked_bbox vec with bbox id " <<  bbox.bbox_id <<  endl;

       }}
       //}

           //cout << "DEBUG marker" << cube_list.markers.size() << "and locked vec "<< locked_box_vec.bbox_id_vec.size() <<endl;
       cube_list.markers.resize(locked_box_vec.bbox_id_vec.size());


        geometry_msgs::Point raytl, raybr;
        line_list.markers[i].points.clear();
        //cube_list.markers[i].points.clear();

        bool fail=false;
        for (int test=0; test<3; test++){
            if(bbTracker_.bbox_State_vect[i].w_corner[0][test] !=bbTracker_.bbox_State_vect[i].w_corner[0][test]  ){
                fail=true;
                break;
            }}
        if (!fail){

            //cout << "CHECK if exist We publish the point corner tl of  " <<bbTracker_.bbox_State_vect[i].w_corner[0].transpose() << "and br world point" << bbTracker_.bbox_State_vect[i].w_corner[3].transpose() << endl;
            line_list.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
            line_list.markers[i].id = i;
            line_list.markers[i].scale.x = 0.1;
            line_list.markers[i].header.frame_id = "world";
            line_list.markers[i].header.stamp = ros::Time::now();
            line_list.markers[i].ns = "my lines_" ;
            line_list.markers[i].action = visualization_msgs::Marker::ADD;
            line_list.markers[i].pose.orientation.w = 1.0;
        line_list.markers[i].color.b = 1.0;
        line_list.markers[i].color.r = 1.0;
        line_list.markers[i].color.g = 1.0;
        line_list.markers[i].color.a = 1.0;
       // raytl.color.b = 1.0;
        raytl.x =bbTracker_.bbox_State_vect[i].w_corner[0][0];
        raytl.y =bbTracker_.bbox_State_vect[i].w_corner[0][1];
        raytl.z =bbTracker_.bbox_State_vect[i].w_corner[0][2];
        line_list.markers[i].points.push_back(ray0);

          line_list.markers[i].points.push_back(raytl);
          marker_pub.publish(line_list);
        }
          fail=false;
          for (int test=0; test<3; test++){
              if(bbTracker_.bbox_State_vect[i].w_corner[3][test] !=bbTracker_.bbox_State_vect[i].w_corner[3][test] ){
                  fail=true;
                  break;
              }}

          if (!fail){
          line_list.markers[i].color.b = 1.0;
          line_list.markers[i].color.r = 0.0;
          line_list.markers[i].color.g = 0.0;
          line_list.markers[i].color.a = 1.0;
          raybr.x =bbTracker_.bbox_State_vect[i].w_corner[3][0];
          raybr.y =bbTracker_.bbox_State_vect[i].w_corner[3][1];
          raybr.z =bbTracker_.bbox_State_vect[i].w_corner[3][2];
          line_list.markers[i].points.push_back(ray0);
          line_list.markers[i].points.push_back(raybr);

        marker_pub.publish(line_list);
}
          //else {

//              line_list.markers[i].points.clear();
//}

            }
}
   int count = locked_box_vec.count;
    for (int i=count; i<locked_box_vec.bbox_id_vec.size(); i++){ //from the last time to the end
    //cout<< "-------------- publishing " << i << "th box on" <<  locked_box_vec.bbox_id_vec.size() << "on id nb" << count << " "
       // << i <<  endl;
               cube_list.markers[i].header.frame_id = "world";
               cube_list.markers[i].header.stamp = ros::Time::now();
               cube_list.markers[i].ns = "my bbox" ; //namspace
               cube_list.markers[i].id = i ;

               cube_list.markers[i].pose.orientation.w = 1.0;

               cube_list.markers[i].type = visualization_msgs::Marker::CUBE;
               cube_list.markers[i].action = visualization_msgs::Marker::ADD;

               // Set the scale of the marker -- 1x1x1 here means 1m on a side
               cube_list.markers[i].scale.x = 0.1;
               cube_list.markers[i].scale.y = 0.7;
               cube_list.markers[i].scale.z = 2;

               // Set the color -- be sure to set alpha to something non-zero!
               cube_list.markers[i].color.r = 0.0f;
               cube_list.markers[i].color.g = 1.0f;
               cube_list.markers[i].color.b = 0.0f;
               cube_list.markers[i].color.a = 1.0;
        //       cube_list.markers[i].pose.orientation.x = 0.0;
        //       cube_list.markers[i].pose.orientation.y = 0.0;
        //       cube_list.markers[i].pose.orientation.z = 0.0;
               cube_list.markers[i].pose.orientation.w = 1.0;


           cube_list.markers[i].pose.position.x = locked_box_vec.locked_boxes[i].center[0]; //locked_box_vec.locked_boxes has size
           cube_list.markers[i].pose.position.y = locked_box_vec.locked_boxes[i].center[1];
           cube_list.markers[i].pose.position.z = locked_box_vec.locked_boxes[i].center[2];
           //cout << "for marker"<< i << "we publish" << cube_list.markers[i].pose.position << endl;
           locked_box_vec.count++;
   }

           cube_pub.publish(cube_list);

           if (locked_box_vec.locked_boxes.size()>10){
            locked_box_vec.count =0;
            locked_box_vec.locked_boxes.clear();
            locked_box_vec.bbox_id_vec.clear();
           }

}
//}else{
 // cout << "no bounding box, delete the rays" << endl;
//   line_list.markers[i].action = visualization_msgs::Marker::DELETE;
//   line_list.markers[i].points.clear();
//   cube_list.markers[i].points.clear();
//   cube_pub.publish(cube_list);
//   marker_pub.publish(line_list);






void publish_extra(const ros::Time& publish_time)
{
  //if(track_image_pub_.getNumSubscribers() > 0){
    display_all = cv_bridge::cvtColor(display_all, sensor_msgs::image_encodings::BGR8);
    cv::Rect ROI(0, 100,display_all->image.cols-100, display_all->image.rows-100);
    cv::Mat image = display_all->image(ROI);
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();


    if (bbTracker_.bbox_State_vect.size()>0 && estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
// the bounding box just detected
    for (int i=0; i<bbTracker_.bbox_State_vect.size(); i++){
    //ROS_INFO_STREAM("publshing the DETECTED bbox in green ===================");

        if (bbTracker_.bb_state_.img_bboxes.list.size()>0){
        cv::Point2f tl = cv::Point2f(bbTracker_.bb_state_.img_bboxes.list[i].xmin, bbTracker_.bb_state_.img_bboxes.list[i].ymin);
        cv::Point2f br = cv::Point2f(bbTracker_.bb_state_.img_bboxes.list[i].xmax, bbTracker_.bb_state_.img_bboxes.list[i].ymax);

        //YOLO DETECTION IN GREEN
        cv::rectangle(image, tl, br,cv::Scalar(0, 255, 0), 2, 8, 0) ; // color in BGR
          //ROS_INFO_STREAM(" \n real tl " << tl << "and  br" << br );
        putText(image, "YOLO", Point2f(300+20*i,100+20*i), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2 );


          //ROS_INFO_STREAM("publshing the TEST bbox in blue ===================");
          Utility::bboxState<float> rayState;
          Utility::bbox<float> test_bbox ;
         // cv::Point2f bbox_corner[4];
          //bbTracker_.project_pixel_to_world(rayState.r_tl ,rayState.r_br , bbTracker_.bb_state_.img_bboxes.list[i] );
//          bbTracker_.project_world_to_pixel(rayState, test_bbox);
//          tl = cv::Point2f(test_bbox.xmin, test_bbox.ymin);
//          br = cv::Point2f(test_bbox.xmax, test_bbox.ymax);
//          bbTracker_.bbox_to_points_cv(test_bbox, bbox_corner );

//          cv::rectangle(display_all->image, tl, br,cv::Scalar(255, 0, 0), 2, 8, 0) ; // color in BGR


}

    //bounding box predicted
    bbTracker_.project_pixel_to_pixel();
    for (unsigned i=0; i<bbTracker_.bbox_State_vect.size(); i++){
        if(bbTracker_.bbox_State_vect[i].nb_detected>4){
        Utility::bbox<float> predicted_bbox ;
         cv::Point2f bbox_corner[4], deduced_corner[4];
         bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].w_corner, predicted_bbox);
         bbTracker_.bbox_to_points_cv(predicted_bbox, deduced_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame

           //DEDUCED FROM CUR DETECTION IN RED
        cv::rectangle(image, deduced_corner[0], deduced_corner[3],cv::Scalar(0, 0, 255), 2, 8, 0) ;
        putText(image, "w_corner", Point2f(200+20*i,100+20*i), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2 );


        //ROS_INFO_STREAM("for original bbox" << bbTracker_.bbox_State_vect[i].bbox_id << "with tl" <<bbox_corner[0] << "and br" << bbox_corner[3]
            // << "and deduced" << deduced_corner[0] <<"br" << deduced_corner[3]);

//        cv::Point2f tl = cv::Point2f(bbTracker_.bbox_State_vect[i].deduced_pixel[0]);
//        cv::Point2f br = cv::Point2f(bbTracker_.bbox_State_vect[i].deduced_pixel[3]);
        bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].locked_corner, predicted_bbox);
        bbTracker_.bbox_to_points_cv(predicted_bbox, deduced_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame

        //DEDUCED FROM LOCKED IN WHITE
        cv::rectangle(image, deduced_corner[0], deduced_corner[3],cv::Scalar(255, 255, 255), 2, 8, 0) ;
          //ROS_INFO_STREAM("==================tl " << tl << "\n br" << br  );

       putText(image, bbTracker_.bbox_State_vect[i].type_detection, Point2f(100+20*i,100+20*i), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,255,255,255), 2 );


       bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].locked_corner_def, predicted_bbox);
       bbTracker_.bbox_to_points_cv(predicted_bbox, deduced_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame

       //DEDUCED FROM LOCKED IN WHITE
       cv::rectangle(image, deduced_corner[0], deduced_corner[3],cv::Scalar(255, 0, 0), 2, 8, 0) ;
}

    for (unsigned int k=0; k<4;k++){
       // ROS_INFO_STREAM("Publish pointcornerof " << bbTracker_.bbox_State_vect[i].bbox_id << ":"
        //                << bbTracker_.bbox_State_vect[i].w_corner[k].transpose() );

    geometry_msgs::Point32 p;
    p.x = bbTracker_.bbox_State_vect[i].w_corner[k][0];
    p.y = bbTracker_.bbox_State_vect[i].w_corner[k][1];
    p.z = bbTracker_.bbox_State_vect[i].w_corner[k][2];
    if (p.x != 0 && p.y != 0 && p.z != 0) {
    point_cloud.points.push_back(p); }
    }
}
}
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();

    track_image_pub_.publish(msg);
  }pub_corner_point.publish(point_cloud);

}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    bbTracker_.frame_count++;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        out_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        optical_result_bridge =  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }else{
        out_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        optical_result_bridge =  cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    }
    ROS_INFO("RECEIVING IMG");

        //bbTracker_.shift_frame(out_img->image); //shift the detected points from prev_frame
    cv::Mat passerelle = optical_result_bridge->image;
    cv::Rect ROI(0, 100,passerelle.cols-100, passerelle.rows-100);
    cv::Mat passerelle_out = passerelle(ROI);
     bbTracker_.shift_all(passerelle_out, passerelle_out);
       sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, passerelle_out).toImageMsg();
       optical_image_pub_.publish(msg);
}

void img_display_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
        display_all = cv_bridge::toCvCopy(img_msg);
}


void sync_callback(const sensor_msgs::ImageConstPtr &img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr &bboxes_ros){

 if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
     return;

    cout <<"RECEIVING " << bboxes_ros->bounding_boxes.size() << "bboxes" << endl; //print the full topic

        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            out_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }else{
            out_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        cv::Rect ROI(0, 100,out_img->image.cols-100, out_img->image.rows-100);
        bbTracker_.cur_frame = out_img->image(ROI);



        //Boundingbox callback
        double img_bboxes_time = bboxes_ros->header.stamp.toSec();
        yolo_img_time = img_msg->header.stamp.toSec();
        yolo_bbox_time = img_msg->header.stamp.toSec();

        //double img_bboxes_time = ros::Time::now().toSec();

        //cout << bboxes_ros.img_w < " is saved in " << bb_state_.img_w_ << endl;
        Utility::imgBboxes<float> bboxes;
        Utility::bbox<float> predicted_bbox, predicted_bbox_def ;
        int added=0;


        for (unsigned int i=0; i< bboxes_ros->bounding_boxes.size(); i++){
            //bbTracker_.img_w_= bboxes_ros->img_w;
            //bbTracker_.img_h_= bboxes_ros->img_h;
            Utility::bbox<float> boundingBox, un_bbox;
            if (bboxes_ros->bounding_boxes.at(i).ymin >100 && bboxes_ros->bounding_boxes.at(i).xmax<552){

            try{
            boundingBox.Class = bboxes_ros->bounding_boxes.at(i).Class ;
            boundingBox.prob  = bboxes_ros->bounding_boxes.at(i).probability ;
            boundingBox.xmin  = bboxes_ros->bounding_boxes.at(i).xmin ;
            boundingBox.ymin = bboxes_ros->bounding_boxes.at(i).ymin-100 ;
            boundingBox.xmax  = bboxes_ros->bounding_boxes.at(i).xmax ;
            boundingBox.ymax  = bboxes_ros->bounding_boxes.at(i).ymax-100 ;
            bboxes.list.push_back(boundingBox);

             // ROS_INFO_STREAM("Size of bounding box " << boundingBox.xmin <<","<< boundingBox.ymin );
            }catch(const std::exception& e){
               // cout << "Wrong bounding box format, skipped" << endl;
            }
            bbTracker_.bb_state_.img_bboxes= bboxes;

            bbTracker_.frame_count =0;
            un_bbox = bbTracker_.undistortPoint(boundingBox);
            //bbTracker_.multiTracker->update( bbTracker_.cur_frame);

     //bbTracker_.bbox_State_vect.clear(); //temporary

            //for every bounding box now detected
                  //for (int i=0; i<bboxes.list.size(); i++){
                        bool tracked = false;
                        bool valid = true;

                        cout << "When receiving bbox speed is" << tmp_V << endl;

                      //check if bbox is already tracked
                      for (unsigned j=0; j<bbTracker_.bbox_State_vect.size(); j++){ //check if really updating
                          if (abs(tmp_V[0])>0.1 ||abs(tmp_V[1])>0.1 ||abs(tmp_V[2])>0.1  ){
                             thresh_IOU = 0.1;
                          }
                          if (abs(tmp_V[0])>0.15 ||abs(tmp_V[1])>0.15 ||abs(tmp_V[2])>0.15  ){
                             valid = false;
                          }
                          bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[j].locked_corner, predicted_bbox);
                          bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[j].locked_corner_def, predicted_bbox_def);

                if(bbTracker_.IOU(boundingBox, bbTracker_.bbox_State_vect[j].cur_detection, thresh_IOU)
                        || bbTracker_.IOU(boundingBox, predicted_bbox, thresh_IOU)
                        || bbTracker_.IOU(boundingBox, predicted_bbox_def, thresh_IOU) ){
                                bbTracker_.bbox_State_vect[j].prev_detection = bbTracker_.bbox_State_vect[j].cur_detection; //update already existing bbox
                                bbTracker_.bbox_State_vect[j].cur_detection = boundingBox;
                                bbTracker_.bbox_State_vect[j].age=0;
                                bbTracker_.bbox_State_vect[j].nb_detected++;
                                bbTracker_.bbox_State_vect[j].time = img_bboxes_time;
                                bbTracker_.bbox_State_vect[j].associated = true;
                                bbTracker_.bbox_State_vect[j].poses_vec.push_back(estimator.Ps[WINDOW_SIZE]);
                                bbTracker_.bbox_State_vect[j].rotations_vec.push_back(estimator.Rs[WINDOW_SIZE]);
                                bbTracker_.bbox_State_vect[j].pixel.push_back(boundingBox);
                                bbTracker_.bbox_State_vect[j].un_pixel.push_back(un_bbox);
                                bbTracker_.bbox_State_vect[j].last_img= bbTracker_.cur_frame;
                                bbTracker_.bbox_State_vect[j].type_detection= "cnn";


                                //add the pose and the pixel values in the sliding window
                                tracked=true;
                                cout << "tracked !! bbox_id is "<<bbTracker_.bbox_State_vect[j].bbox_id   <<" tracked for:"<<
                                        bbTracker_.bbox_State_vect[j].nb_detected<<" detection and its the bbox nb"<<j <<"in the vec  ,and corner id are:" << bbTracker_.bbox_State_vect[j].feature_id[0] << ", " << bbTracker_.bbox_State_vect[j].feature_id[1] << ", " << bbTracker_.bbox_State_vect[j].feature_id[2] <<
                                        ", " <<bbTracker_.bbox_State_vect[j].feature_id[3] << "with" << bbTracker_.bbox_State_vect[j].poses_vec.size() << "poses "<< endl;
                                break; //quit the comparaison with the previous bbox updated

                      }
                       } //we dealt with all the previous saved bbox
                       if((tracked==false || bbTracker_.bbox_State_vect.size() == 0) && valid){ //if the box has not been used for update we add as a new detection

                       //Box never has been tracked we will augment the bboxState
                       Utility::bboxState<float> rayState; //create a bbox state
                       bbTracker_.project_pixel_to_world(rayState.r_tl ,rayState.r_br , boundingBox ); //init the ray
                       rayState.bbox_id = bbTracker_.id_count;
                       rayState.cur_detection = boundingBox;
                       rayState.prev_detection = boundingBox;
                       rayState.age=0;
                       rayState.nb_detected=0;
                       rayState.time= img_bboxes_time;
                       bbTracker_.id_count++;
                       rayState.associated = false;
                       rayState.poses_vec.push_back(estimator.Ps[WINDOW_SIZE]);
                       rayState.rotations_vec.push_back(estimator.Rs[WINDOW_SIZE]);
                       rayState.pixel.push_back(boundingBox);
                       rayState.un_pixel.push_back(un_bbox);
                       rayState.last_img= bbTracker_.cur_frame;
                       rayState.type_detection= "cnn";
                       rayState.lock_proba = 0;


                       //rayState.w_corner= {corner_init,corner_init,corner_init,corner_init};
                        added++;

                       //rayState.pixel.push_back(boundingBox);
                       //add the firstpose and the pixel values in the sliding window
                       bbTracker_.bbox_State_vect.push_back(rayState); //we push the

            }
}
}

//                  for (auto bbox_added : reserve){
//                      bbTracker_.bbox_State_vect.push_back(bbox_added);
//                  }

                  cout << "After adding" <<  added<<", now we have" <<bbTracker_.bbox_State_vect.size() << "bounding boxes ";

                    for (unsigned j=0; j<bbTracker_.bbox_State_vect.size(); j++)
                    {
                        cout << "with the id" << bbTracker_.bbox_State_vect[j].bbox_id << endl;
                        //they could not be updated then we can shift their values with optical flow only
                             if (!(bbTracker_.bbox_State_vect[j].time == img_bboxes_time) )

                             { //update only once
                                 //cout << "update old one" <<  bbTracker_.bbox_State_vect[j].bbox_id<< "time" << img_bboxes_time << "and" <<bbTracker_.bbox_State_vect[j].time << endl;
                             bbTracker_.bbox_State_vect[j].age++; //case the bbox was not matching we add up once its age if not done
                             //bbTracker_.bbox_State_vect[j].prev_detection = bbTracker_.bbox_State_vect[j].cur_detection ;
                             bbTracker_.bbox_State_vect[j].associated = false;
                          }
                  }

bbTracker_.prev_frame = bbTracker_.cur_frame;

}




void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{

    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }

    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{

    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{

    printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second; //for the image/pt cloud feature vector
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first) //for all imu before this image
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                ROS_DEBUG("imu timestamp is %f \n", imu_msg->header.stamp.toSec());
                if (t <= img_t) //if the time of imu is less than the considered image + delay
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else //the IMU is ??
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x; //unit ray in the sphere
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image; //contain id_feature as a key and vector of <CAM_ID, xyzuvvv>
           // ROS_INFO("point size %u \n", img_msg->points.size());
            for (unsigned int i = 0; i < img_msg->points.size(); i++) //for all the points in the poincloud
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM; //id as detected
                int camera_id = v % NUM_OF_CAM; // id as detected
                double x = img_msg->points[i].x; //undistorded and on sphere
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i]; //pixel values
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity); //can create

                //std::cout  << "feature_id" << feature_id << "v" << v <<"camera_id" << "x and y " << x << "y" << y << "p_u" << p_u << "p_v" << p_v  << "velocity_x" << velocity_x << std::endl;
                //ROS_DEBUG("in estinmator node feature id",  feature_id, " and v", v, " p_u " , p_u," p_v ", p_v );

            }


            //all feature in image will be added in estimator.feature or updated if id already existant
            estimator.processImage(image, img_msg->header);
            bbTracker_.update_id(image); //match id and bboxState with no id yet
            bbTracker_.reproj(0.4);


            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            publish_rviz(img_msg->header.stamp);
            publish_extra(img_msg->header.stamp);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();


        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)    {
           // cout << "in process We give the depth to the corner" ;
           // bool distance= false;
            for (unsigned j=0; j<bbTracker_.bbox_State_vect.size(); j++){
                //distance = true;
                for (unsigned int k=0; k<4;k++){
                     size_t feature_id_bb = bbTracker_.bbox_State_vect[j].feature_id[k]; //retrieve the feature for the corner
                    if (feature_id_bb>0){
                        auto it = find_if(estimator.f_manager.feature.begin(),
                                          estimator.f_manager.feature.end(), [feature_id_bb](const FeaturePerId &it) //if this feature already in feature
                                          {
                            return it.feature_id == feature_id_bb;
                                          });

                        if (it == estimator.f_manager.feature.end()) //either add new feature
                        {
                            //feature not found it does not exist anymore raise error
                            bbTracker_.bbox_State_vect[j].feature_id[k]=0;

                        } else if (it->feature_id == feature_id_bb && it->estimated_depth>0) //either update the feature
                        {
                            //retrieve the depth
                            bbTracker_.bbox_State_vect[j].depth[k] = it->estimated_depth;
                            Vector3d pts_i = it->feature_per_frame[0].point * it->estimated_depth;
                            int imu_i = it->start_frame;
                            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                           + estimator.Ps[imu_i];
                            bbTracker_.bbox_State_vect[j].w_corner[k] = {w_pts_i(0),w_pts_i(1),w_pts_i(2)};
//                            cout << "We give the depth "<< it->estimated_depth << "to the corner" << k
//                                 << " of the bbox id" <<bbTracker_.bbox_State_vect[j].bbox_id << "with feature id" <<feature_id_bb << endl;
                            //cout << k << " the 3D value" << bbTracker_.bbox_State_vect[j].w_corner[k] << endl; ;

                        }
                    }
                }

            }

           // if (distance)
                //cout << " from the feature distance.  "<< std::endl;

            update();
        }
//        if (img_time>yolo_img_time){
//        cout << "TIME diff is " << img_time-yolo_img_time << "BBOX and img" << img_time-yolo_bbox_time << endl;
//        } else {
//            cout << "TIME NEGATIVE img is" << img_time << "and yolo is " << yolo_img_time << endl;
//        }
        bbTracker_.update_pose(tmp_P, tmp_Q, estimator.ric[NUM_OF_CAM-1], estimator.tic[NUM_OF_CAM-1]); //check the value
        ROS_DEBUG("after update bb");

        m_state.unlock();
        m_buf.unlock();
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
   // image_transport::ImageTransport it(n);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    std::string config_file;
    n.getParam("config_file", config_file);

    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    //optical_image_pub_ = it.advertise("optical_result", 2000);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_image_tracked = n.subscribe("/feature_tracker/feature_img", 2000, img_display_callback);
    ros::Subscriber sub_cam_image = n.subscribe("/hummingbird/vi_sensor/left/image_raw", 2000, img_callback);
    pub_corner_point = n.advertise<sensor_msgs::PointCloud>("corner_point", 1000);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    //ros::Subscriber bb_sub_= n.subscribe("bounding_boxes", 2000, boundingboxesCallback);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("lines", 2000);
    cube_pub = n.advertise<visualization_msgs::MarkerArray>("cube_bbox", 2000);
    //track_image_pub_ = it.advertise("track_overlay_image", 1);
    track_image_pub_ = n.advertise<sensor_msgs::Image>("track_overlay_image",2000); //for the bbox check
    optical_image_pub_ = n.advertise<sensor_msgs::Image>("optical_result",2000); //for the bbox check

    message_filters::Subscriber<sensor_msgs::Image> sub_darknet_image(n, "/detection_image", 2000);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub_(n,"/bounding_boxes", 2000);

    message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes>
            sync(sub_darknet_image, bb_sub_, 2000);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2));

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
