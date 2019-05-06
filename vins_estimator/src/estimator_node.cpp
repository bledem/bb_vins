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
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/tracking.hpp>
#include <dlib/optimization/max_cost_assignment.h>
#include <dlib/optimization.h>
#include "utility/Hungarian.h"




#include "utility/utility.h"


#include "estimator.h"
#include "tracker.h"

#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;
bbTracker_t bbTracker_;
visualization_msgs::MarkerArray cube_list, temp_cube_list, line_list;
ros::Publisher marker_pub, track_image_pub_, cube_pub, temp_cube_pub, optical_image_pub_ ;
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
Eigen::Vector3d tmp_V_delayed;

Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d un_acc_prev, un_acc_nxt ;
Eigen::Vector3d gyr_0;
int acc_delay=0;
bool init_feature = 0;
bool init_imu = 1;
bool speed_peak=false;
bool deccel = false;
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
    un_acc_prev = un_acc;
    acc_delay++;
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

    tmp_V_delayed = estimator.Vs[WINDOW_SIZE-2];



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

  //     }

          // cout << "DEBUG marker" << cube_list.markers.size() << "and locked vec "<< locked_box_vec.bbox_id_vec.size() <<endl;
       cube_list.markers.resize(bbTracker_.bbox_State_vect.size()); //suspicious
       temp_cube_list.markers.resize(bbTracker_.bbox_State_vect.size()); //suspicious


        geometry_msgs::Point raytl, raybr;
        line_list.markers[i].points.clear();
        //cube_list.markers[i].points.clear();

        bool fail=false;
        for (int test=0; test<3; test++){
            if(bbTracker_.bbox_State_vect[i].w_corner[0][test] !=bbTracker_.bbox_State_vect[i].w_corner[0][test] || bbTracker_.bbox_State_vect[i].w_corner[0][test]<0.01  ){
                fail=true;
                break;
            }}
//        if (!fail){

//            cout << "CHECK if exist We publish the point corner tl of  " <<bbTracker_.bbox_State_vect[i].w_corner[0].transpose() << "and br world point" << bbTracker_.bbox_State_vect[i].w_corner[3].transpose() << endl;
//            line_list.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
//            line_list.markers[i].id = i;
//            line_list.markers[i].scale.x = 0.1;
//            line_list.markers[i].header.frame_id = "world";
//            line_list.markers[i].header.stamp = ros::Time::now();
//            line_list.markers[i].ns = "my lines_" ;
//            line_list.markers[i].action = visualization_msgs::Marker::ADD;
//            line_list.markers[i].pose.orientation.w = 1.0;
//        line_list.markers[i].color.b = 1.0;
//        line_list.markers[i].color.r = 1.0;
//        line_list.markers[i].color.g = 1.0;
//        line_list.markers[i].color.a = 1.0;
//       // raytl.color.b = 1.0;
//        raytl.x =bbTracker_.bbox_State_vect[i].w_corner[0][0];
//        raytl.y =bbTracker_.bbox_State_vect[i].w_corner[0][1];
//        raytl.z =bbTracker_.bbox_State_vect[i].w_corner[0][2];
//        line_list.markers[i].points.push_back(ray0);

//          line_list.markers[i].points.push_back(raytl);
//          marker_pub.publish(line_list);
//        }
//          fail=false;
//          for (int test=0; test<3; test++){
//              if(bbTracker_.bbox_State_vect[i].w_corner[3][test] !=bbTracker_.bbox_State_vect[i].w_corner[3][test] ){
//                  fail=true;
//                  break;
//              }}

//          if (!fail){
//          line_list.markers[i].color.b = 1.0;
//          line_list.markers[i].color.r = 0.0;
//          line_list.markers[i].color.g = 0.0;
//          line_list.markers[i].color.a = 1.0;
//          raybr.x =bbTracker_.bbox_State_vect[i].w_corner[3][0];
//          raybr.y =bbTracker_.bbox_State_vect[i].w_corner[3][1];
//          raybr.z =bbTracker_.bbox_State_vect[i].w_corner[3][2];
//          line_list.markers[i].points.push_back(ray0);
//          line_list.markers[i].points.push_back(raybr);

//        marker_pub.publish(line_list);
//}
          //else {

//              line_list.markers[i].points.clear();
//}




//    cout<< "-------------- publishing " << i << "th box on" <<  bbTracker_.bbox_State_vect.size() <<
//           "def lock state is " << bbTracker_.bbox_State_vect[i].locked_def << endl;
   std::array<float,3> visitor_size={0.1, 0.7, 2};
   std::array<float,3> chair_size={0.1, 0.7, 1}; //x is cube_deepness, y is cube_y, z is cube_x
   std::array<float,3> choice;

   if (bbTracker_.bbox_State_vect[i].locked_def== true){
               cube_list.markers[i].header.frame_id = "world";
               cube_list.markers[i].header.stamp = ros::Time::now();
               cube_list.markers[i].ns = "my bbox" ; //namspace
               cube_list.markers[i].id = i ;

               cube_list.markers[i].pose.orientation.w = 1.0;

               cube_list.markers[i].type = visualization_msgs::Marker::CUBE;
               cube_list.markers[i].action = visualization_msgs::Marker::ADD;
                float x_lenght=(bbTracker_.bbox_State_vect[i].locked_corner_def[0][0]+bbTracker_.bbox_State_vect[i].locked_corner_def[3][0]);
                float y_lenght= bbTracker_.bbox_State_vect[i].locked_corner_def[0][1]+bbTracker_.bbox_State_vect[i].locked_corner_def[3][1];
                float z_lenght= bbTracker_.bbox_State_vect[i].locked_corner_def[0][2]+bbTracker_.bbox_State_vect[i].locked_corner_def[3][2];
                //cout << "check lenght x, y, z" << x_lenght << " " << y_lenght << " " << z_lenght << endl;
               // Set the scale of the marker -- 1x1x1 here means 1m on a side

                if (bbTracker_.bbox_State_vect[i].class_ == "visitor"){
                    choice = visitor_size;
                }else{
                    choice = chair_size;
                }
               cube_list.markers[i].scale.x = choice[0];
               cube_list.markers[i].scale.y = choice[1];
               cube_list.markers[i].scale.z = choice[2];

               // Set the color -- be sure to set alpha to something non-zero!
               cube_list.markers[i].color.r = 0.0f;
               cube_list.markers[i].color.g = 1.0f;
               cube_list.markers[i].color.b = 0.0f;
               cube_list.markers[i].color.a = 1.0;
        //       cube_list.markers[i].pose.orientation.x = 0.0;
        //       cube_list.markers[i].pose.orientation.y = 0.0;
        //       cube_list.markers[i].pose.orientation.z = 0.0;
               cube_list.markers[i].pose.orientation.w = 1.0;


           cube_list.markers[i].pose.position.x = x_lenght/2; //locked_box_vec.locked_boxes has size
           cube_list.markers[i].pose.position.y = y_lenght/2;
           cube_list.markers[i].pose.position.z = z_lenght/2;
           //cout << "for marker"<< i << "we publish" << cube_list.markers[i].pose.position << "end" << endl;
           temp_cube_list.markers[i].header.frame_id = "world";
           temp_cube_list.markers[i].header.stamp = ros::Time::now();
           temp_cube_list.markers[i].ns = "my temp bbox" ; //namspace
           temp_cube_list.markers[i].id = i ;
\
           temp_cube_list.markers[i].pose.orientation.w = 1.0;

           temp_cube_list.markers[i].type = visualization_msgs::Marker::CUBE;
           temp_cube_list.markers[i].action = visualization_msgs::Marker::ADD;

           // Set the scale of the marker -- 1x1x1 here means 1m on a side
           temp_cube_list.markers[i].scale = cube_list.markers[i].scale;

           temp_cube_list.markers[i].color.r = 1.0f;
           temp_cube_list.markers[i].color.g = 0.0f;
           temp_cube_list.markers[i].color.b = 0.0f;
           temp_cube_list.markers[i].color.a = 1.0;
           temp_cube_list.markers[i].pose.position.x = (bbTracker_.bbox_State_vect[i].locked_corner[0][0]+bbTracker_.bbox_State_vect[i].locked_corner[3][0])/2;
            temp_cube_list.markers[i].pose.position.y = (bbTracker_.bbox_State_vect[i].locked_corner[0][1]+bbTracker_.bbox_State_vect[i].locked_corner[3][1])/2;
           temp_cube_list.markers[i].pose.position.z = (bbTracker_.bbox_State_vect[i].locked_corner[0][2]+bbTracker_.bbox_State_vect[i].locked_corner[3][2])/2;
   //}
           cube_pub.publish(cube_list);
           temp_cube_pub.publish(temp_cube_list);
}

   }
//   }else{
//     cout << "no bounding box, delete the rays" << endl;
//   //   line_list.markers[i].action = visualization_msgs::Marker::DELETE;
//   //   line_list.markers[i].points.clear();
//       for (unsigned i =0; i<bbTracker_.bbox_State_vect.size(); i++)
//      temp_cube_list.markers[i].points.clear();
//      temp_cube_pub.publish(cube_list);
//   //   marker_pub.publish(line_list);
//   }
}
}







void publish_extra(const ros::Time& publish_time)
{

    bool show_YOLO=true;
    bool show_locked=false;
    bool show_w_corner=false;




  //if(track_image_pub_.getNumSubscribers() > 0){
    display_all = cv_bridge::cvtColor(display_all, sensor_msgs::image_encodings::BGR8);
    //cv::Rect ROI(0, 100,display_all->image.cols-100, display_all->image.rows-100);
    cv::Mat image = display_all->image;
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();


    if (bbTracker_.bbox_State_vect.size()>0 && estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
// the bounding box just detected
        for (int b=0; b<bbTracker_.bb_state_.img_bboxes.list.size(); b++){
        cv::Point2f tl = cv::Point2f(bbTracker_.bb_state_.img_bboxes.list[b].xmin, bbTracker_.bb_state_.img_bboxes.list[b].ymin);
        cv::Point2f br = cv::Point2f(bbTracker_.bb_state_.img_bboxes.list[b].xmax, bbTracker_.bb_state_.img_bboxes.list[b].ymax);

        //YOLO DETECTION IN GREEN
        if (show_YOLO){
        cv::rectangle(image, tl, br,cv::Scalar(0, 255, 0), 2, 8, 0) ; // color in BGR
          //ROS_INFO_STREAM(" \n real tl " << tl << "and  br" << br );
        putText(image, "YOLO", Point2f(300+20*b,100+20*b), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2 );
}

        if(acc_delay>4){
            acc_delay=0;
            un_acc_nxt=un_acc_prev;}
        if((abs(tmp_V[0])>1 || abs(tmp_V[1])>1 || abs(tmp_V[2])>1 )){ //if the speed is high
            speed_peak=true;
            cout << " PEAK===" << tmp_V<< endl;

        }else if ((abs(tmp_V[0])<0.3 && abs(tmp_V[1])<0.3&& abs(tmp_V[2])<0.3 ) && speed_peak ){ //and the robot deccelearate
            speed_peak=false;
            deccel=true;

            cout << "DECCEL" << endl;
        }
        if((abs(tmp_V[0])<0.04&& abs(tmp_V[1])<0.04 && abs(tmp_V[2])<0.05 ) && deccel ){
            deccel=false;
            cout << "END  DECCEL" << endl;
            speed_peak=false;

        }


}
        //ROS_INFO_STREAM("Drawing" << bbTracker_.bbox_State_vect.size()<< "bounding boxes");

    for (int i=0; i<bbTracker_.bbox_State_vect.size(); i++){
    //bounding box predicted
    bbTracker_.project_pixel_to_pixel();
    for (unsigned int i=0; i<bbTracker_.bbox_State_vect.size(); i++){
        if(bbTracker_.bbox_State_vect[i].nb_detected>4){
        Utility::bbox<float> predicted_bbox ;
         cv::Point2f bbox_corner[4], deduced_corner[4];
         bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].w_corner, predicted_bbox);
         bbTracker_.bbox_to_points_cv(predicted_bbox, deduced_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame

           //DEDUCED FROM CUR DETECTION IN RED
         if (show_w_corner){

        cv::rectangle(image, deduced_corner[0], deduced_corner[3],cv::Scalar(0, 0, 255), 2, 8, 0) ;
        putText(image, "w_corner", Point2f(200+20*i,200+20*i), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255), 2 );
}

        //ROS_INFO_STREAM("for original bbox" << bbTracker_.bbox_State_vect[i].bbox_id << "with tl" <<bbox_corner[0] << "and br" << bbox_corner[3]
            // << "and deduced" << deduced_corner[0] <<"br" << deduced_corner[3]);

//        cv::Point2f tl = cv::Point2f(bbTracker_.bbox_State_vect[i].deduced_pixel[0]);
//        cv::Point2f br = cv::Point2f(bbTracker_.bbox_State_vect[i].deduced_pixel[3]);
        bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].locked_corner, predicted_bbox);

        //if speed to high we take the CNN width but with the position of the reproj for the final tracking
if (abs(tmp_V[0])>0.1 || abs(tmp_V[1])>0.1 || abs(tmp_V[2])>0.1)
            bbTracker_.reproj(0.4);
if (abs(tmp_V[0])>0.3 || abs(tmp_V[1])>0.3 || abs(tmp_V[2])>0.3 || deccel)
            bbTracker_.reproj(0);

        if (abs(tmp_V[0])>0.03 || abs(tmp_V[1])>0.03 || abs(tmp_V[2])>0.03 || deccel && bbTracker_.bbox_State_vect[i].lock_proba>=15){
            float x_avg = (bbTracker_.bbox_State_vect[i].cur_detection.xmin + bbTracker_.bbox_State_vect[i].cur_detection.xmax)/2;
            float y_avg = (bbTracker_.bbox_State_vect[i].cur_detection.ymin + bbTracker_.bbox_State_vect[i].cur_detection.ymax)/2;

            bbTracker_.bbox_State_vect[i].final_bb[0] = cv::Point2f(x_avg-bbTracker_.bbox_State_vect[i].w_l.first/2, y_avg-bbTracker_.bbox_State_vect[i].w_l.second/2) ; //tl
            bbTracker_.bbox_State_vect[i].final_bb[1] = cv::Point2f(x_avg+bbTracker_.bbox_State_vect[i].w_l.first/2, y_avg-bbTracker_.bbox_State_vect[i].w_l.second/2) ; //tr
            bbTracker_.bbox_State_vect[i].final_bb[2] = cv::Point2f(x_avg-bbTracker_.bbox_State_vect[i].w_l.first/2, y_avg+bbTracker_.bbox_State_vect[i].w_l.second/2) ; //
            bbTracker_.bbox_State_vect[i].final_bb[3] = cv::Point2f(x_avg+bbTracker_.bbox_State_vect[i].w_l.first/2, y_avg+bbTracker_.bbox_State_vect[i].w_l.second/2) ;
            cv::rectangle(image, bbTracker_.bbox_State_vect[i].final_bb[0], bbTracker_.bbox_State_vect[i].final_bb[3],cv::Scalar(255, 255, 255), 2, 8, 0) ;
          //  cout << "%%%%%%%%%% From cur detection" << bbTracker_.bbox_State_vect[i].final_bb[0] << bbTracker_.bbox_State_vect[i].final_bb[1] << bbTracker_.bbox_State_vect[i].final_bb[2] << "with widt/lenght" <<
         //          bbTracker_.bbox_State_vect[i].w_l.first/2 << " "<<bbTracker_.bbox_State_vect[i].w_l.second/2 <<  endl;
           // cout << "debug cur detec after0" << endl;

        }else{

            bbTracker_.bbox_to_points_cv(bbTracker_.bbox_State_vect[i].cur_detection, bbox_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame
            cv::rectangle(image, bbox_corner[0], bbox_corner[3],cv::Scalar(255, 255, 255), 2, 8, 0) ;

        }


        //DEDUCED FROM LOCKED IN WHITE

       putText(image, bbTracker_.bbox_State_vect[i].type_detection, Point2f(100+(20*i),100+(20*i)), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,0,0), 2 );
       putText(image, std::to_string(bbTracker_.bbox_State_vect[i].bbox_id), Point2f(100,300+(30*i)), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,255,255,255), 2 );
       putText(image, bbTracker_.bbox_State_vect[i].class_, Point2f(300,300+(30*i)), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,0,0), 2 );

       putText(image, "V_x=" + std::to_string(tmp_V[0]).substr (0,4), Point2f(10,100), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255,255), 2 );
       putText(image, "V_y=" + std::to_string(tmp_V[1]).substr (0,4), Point2f(10,200), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255,255), 2 );
       putText(image, "V_z=" +  std::to_string(tmp_V[2]).substr (0,4), Point2f(10,300), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255,255), 2 );

//       putText(image, "x=" + std::to_string(tmp_P[0]), Point2f(400,150), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,255,255), 2 );
//       putText(image, "y=" + std::to_string(tmp_P[1]), Point2f(400,250), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,255,255), 2 );
//       putText(image, "z=" +  std::to_string(tmp_P[2]), Point2f(400,350), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,255,255), 2 );

        if (show_locked){
       bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[i].locked_corner_def, predicted_bbox);
       bbTracker_.bbox_to_points_cv(predicted_bbox, deduced_corner ); //deduced corner takes the bbox_state.w_corner[0] and project them in the current cam_image frame
       //DEDUCED FROM LOCKED IN WHITE
       cv::rectangle(image, deduced_corner[0], deduced_corner[3],cv::Scalar(255, 0, 0), 2, 8, 0) ;
}
        //check acceleration between frame of sliding window
//        if(deccel==true){
//            putText(image, "deccel" , Point2f(200, 30), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,255,255), 2 );

//        }
//        if((un_acc_nxt-un_acc_prev).norm()>0.1 || (un_acc_prev-un_acc_nxt).norm()>0.1 ){

//            putText(image, "acc" +  std::to_string((un_acc_prev).norm()) + " " + std::to_string((estimator.Vs[WINDOW_SIZE-4]-tmp_V).norm()), Point2f(30, 30), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,255,255), 2 );

//        }
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
//cout<< "Frame count is " << bbTracker_.frame_count << endl;

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
    //ROS_INFO("RECEIVING IMG");

        //bbTracker_.shift_frame(out_img->image); //shift the detected points from prev_frame
    cv::Mat passerelle = optical_result_bridge->image;
    //cv::Rect ROI(0, 100,passerelle.cols-100, passerelle.rows-100);
    //cv::Mat passerelle_out = passerelle(ROI);
     cv::Point2f bbox_corner[4];
     if (abs((tmp_V[0])<0.05 && abs(tmp_V[1])<0.05 && abs(tmp_V[2])<0.05) ) //|| (abs(tmp_V[0])>0.01 || abs(tmp_V[1])>0.01 || abs(tmp_V[2])>0.01))
        bbTracker_.shift_all(passerelle, passerelle);
       sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, passerelle).toImageMsg();
       optical_image_pub_.publish(msg);
}

void img_display_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
        display_all = cv_bridge::toCvCopy(img_msg);
}


void sync_callback(const sensor_msgs::ImageConstPtr &img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr &bboxes_ros){
bool first=false;
//cout <<"RECEIVING " << bboxes_ros->bounding_boxes.size() << " bboxes in callback " << endl; //print the full topic

 if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
     return;
 if (bbTracker_.bbox_State_vect.size()==0)
     first=true;
 std::vector< std::vector<double> > iou_cost;
 vector<int> assignment, left;



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
       // cv::Rect ROI(0, 100,out_img->image.cols-100, out_img->image.rows-100);
        bbTracker_.cur_frame = out_img->image;



        //Boundingbox callback
        double img_bboxes_time = bboxes_ros->header.stamp.toSec();
        yolo_img_time = img_msg->header.stamp.toSec();
        yolo_bbox_time = img_msg->header.stamp.toSec();

        //double img_bboxes_time = ros::Time::now().toSec();

        //cout << bboxes_ros.img_w < " is saved in " << bb_state_.img_w_ << endl;
        Utility::imgBboxes<float> bboxes;
        Utility::bbox<float> predicted_bbox, predicted_bbox_def ;
        int added=0;

        bool valid = true;
//cv::Mat  iou_mat= cv::Mat::zeros(cv::Size(bboxes_ros->bounding_boxes.size(),bbTracker_.bbox_State_vect.size()),CV_64FC1 );


        HungarianAlgorithm HungAlgo;
        //cout << "SPEED" << tmp_V.transpose() <<  "delayed " << tmp_V_delayed << endl;
        if (abs(tmp_V_delayed[0])>0.1 ||abs(tmp_V_delayed[1])>0.1 ||abs(tmp_V_delayed[2])>0.1  ){
           thresh_IOU = 0.1;
        }
        if (abs(tmp_V_delayed[0])>0.15 ||abs(tmp_V_delayed[1])>0.15 ||abs(tmp_V_delayed[2])>0.15  ){
           valid = false;
        }
        for (unsigned int i=0; i< bboxes_ros->bounding_boxes.size(); i++){
            //bbTracker_.img_w_= bboxes_ros->img_w;
            //bbTracker_.img_h_= bboxes_ros->img_h;

            Utility::bbox<float> boundingBox, un_bbox;
            try{

            boundingBox.Class = bboxes_ros->bounding_boxes.at(i).Class ;
            boundingBox.prob  = bboxes_ros->bounding_boxes.at(i).probability ;
            boundingBox.xmin  = bboxes_ros->bounding_boxes.at(i).xmin ;
            boundingBox.ymin = bboxes_ros->bounding_boxes.at(i).ymin ;
            boundingBox.xmax  = bboxes_ros->bounding_boxes.at(i).xmax ;
            boundingBox.ymax  = bboxes_ros->bounding_boxes.at(i).ymax ;
            bboxes.list.push_back(boundingBox);

             // ROS_INFO_STREAM("Size of bounding box " << boundingBox.xmin <<","<< boundingBox.ymin );
            }catch(const std::exception& e){
               // cout << "Wrong bounding box format, skipped" << endl;
            }
            bbTracker_.bb_state_.img_bboxes= bboxes;
         //   if (boundingBox.Class != "chair")
         //       return;
                boundingBox = bboxes.list[i];
            bbTracker_.frame_count =0;
            un_bbox = bbTracker_.undistortPoint(boundingBox);
            int width = boundingBox.xmax-boundingBox.xmin;
            int lenght = boundingBox.ymax-boundingBox.ymin;
            //bbTracker_.multiTracker->update( bbTracker_.cur_frame);

     //bbTracker_.bbox_State_vect.clear(); //temporary

            //for every bounding box now detected
                  //for (int i=0; i<bboxes.list.size(); i++){
                        bool tracked = false;


            //cout <<"for bbox" << i <<endl;
                      //check if bbox is already tracked
                      if (!first) {
                      for (unsigned j=0; j<bbTracker_.bbox_State_vect.size(); j++){ //check if really updating

                          bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[j].locked_corner, predicted_bbox);
                          bbTracker_.project_world_to_pixel(bbTracker_.bbox_State_vect[j].locked_corner_def, predicted_bbox_def);

                if(bbTracker_.IOU(boundingBox, bbTracker_.bbox_State_vect[j].cur_detection, thresh_IOU)
                        || bbTracker_.IOU(boundingBox, predicted_bbox, thresh_IOU)
                        || bbTracker_.IOU(boundingBox, predicted_bbox_def, thresh_IOU) ){
                                bbTracker_.update_bbox_state(j, img_bboxes_time, estimator.Ps[WINDOW_SIZE], estimator.Rs[WINDOW_SIZE], boundingBox,un_bbox);
                   //add the pose and the pixel values in the sliding window
                                tracked=true;
                                cout << "tracked !! bbox_id is "<<bbTracker_.bbox_State_vect[j].bbox_id   <<" tracked for:"<<
                                        bbTracker_.bbox_State_vect[j].nb_detected<<" detection and its the bbox nb"<<j <<"in the vec  ,and corner id are:" << bbTracker_.bbox_State_vect[j].feature_id[0] << ", " << bbTracker_.bbox_State_vect[j].feature_id[1] << ", " << bbTracker_.bbox_State_vect[j].feature_id[2] <<
                                        ", " <<bbTracker_.bbox_State_vect[j].feature_id[3] << "with" << bbTracker_.bbox_State_vect[j].poses_vec.size() << "poses "<< endl;
                                break; //quit the comparaison with the previous bbox updated

                      }
                       }  thresh_IOU = 0.4; //threshback to normal

                      }
                      if (tracked==false) { //unmatched by first method
                           left.push_back(i);
                           //cout << "left add" << i << " is unmatched " <<endl;
                      }


           // std::vector<std::vector<double>> matrix(10, std::vector<double>(10, 0.0));
                std::vector<double> temp;
                for (unsigned j=0; j<bbTracker_.bbox_State_vect.size(); j++){ //check if really updating
                    temp.push_back(bbTracker_.IOU_out(boundingBox, bbTracker_.bbox_State_vect[j].cur_detection, thresh_IOU));
                    //cout << "bbox id " << bbTracker_.bbox_State_vect[j].bbox_id << " has with bbox" << i << " IOU=" <<  bbTracker_.IOU_out(boundingBox, bbTracker_.bbox_State_vect[j].cur_detection, thresh_IOU) << endl;
    }

                iou_cost.push_back(temp); //one elt is one bounding box

}

//                  for (auto bbox_added : reserve){
//                      bbTracker_.bbox_State_vect.push_back(bbox_added);
//                  }
            // 2nd check with hungarian algo to check To find out the best assignment of people to jobs we just need to call this function.

                if (iou_cost.size()>0){
               double cost = HungAlgo.Solve(iou_cost, assignment);
               //for (unsigned int x = 0; x < iou_cost.size(); x++)
//                                if (assignment[x]>=0){
//                               std::cout << "bbox detect at index " << x << " is associated to tracker index " << assignment[x] <<" with tracker id "<< bbTracker_.bbox_State_vect[x].bbox_id << endl;
//                                } else {
                                    //cout << "bbox detect  at index" << x << " assignment failed" << assignment[x] << endl;
              //}

        for (int j=0; j<bbTracker_.bbox_State_vect.size(); j++)
        {
            auto it = find(assignment.begin(),
                              assignment.end(), j); //we associated this tracker to a box

            if (it != assignment.end() && !(bbTracker_.bbox_State_vect[j].time == img_bboxes_time) && valid) //if this tracker has not been matched
            {
                int x = std::distance( assignment.begin(), it ); //we take the bbox index for this matched tracker
                //cout << "unmatched tracker saved by Hungarian with bbox_id: " << bbTracker_.bbox_State_vect[j].bbox_id << "detec_id index is "<< x <<  endl;
                bbTracker_.update_bbox_state(j, img_bboxes_time, estimator.Ps[WINDOW_SIZE], estimator.Rs[WINDOW_SIZE], bboxes.list[x], bbTracker_.undistortPoint(bboxes.list[x]));

                auto it_left = find(left.begin(),
                                  left.end(), x); //check if the x_index of the bbox has been saved in left
                int x_left = std::distance( left.begin(), it_left ); //we take the bbox index for this matched tracker
                if (left.size()> x_left){
                //cout << "it_left value of the bbox at place "<< x_left <<" and left size is" << left.size() << endl;
                left.erase(it_left);
                }

            }

        }
                }
        for ( int i=0; i< left.size(); i++){

        //we dealt with all the previous saved bbox
         if( valid){ //if the box has not been used for update we add as a new detection

         //Box never has been tracked we will augment the bboxState
         Utility::bboxState<float> rayState; //create a bbox state
         bbTracker_.project_pixel_to_world(rayState.r_tl ,rayState.r_br , bboxes.list[i] ); //init the ray
         rayState.bbox_id = bbTracker_.id_count;
         rayState.cur_detection = bboxes.list[i];
         rayState.prev_detection = bboxes.list[i];
         rayState.age=0;
         rayState.nb_detected=0;
         rayState.time= img_bboxes_time;
         bbTracker_.id_count++;
         rayState.associated = false;
         rayState.poses_vec.push_back(estimator.Ps[WINDOW_SIZE]);
         rayState.rotations_vec.push_back(estimator.Rs[WINDOW_SIZE]);
         rayState.pixel.push_back(bboxes.list[i]);
         rayState.un_pixel.push_back(bbTracker_.undistortPoint(bboxes.list[i]));
         rayState.last_img= bbTracker_.cur_frame;
         rayState.type_detection= "cnn";
         rayState.final_bb[0] =  cv::Point2f(bboxes.list[i].xmin, bboxes.list[i].ymin);
         rayState.final_bb[1] = cv::Point2f(bboxes.list[i].xmax, bboxes.list[i].ymin);
         rayState.final_bb[2] =  cv::Point2f(bboxes.list[i].xmin, bboxes.list[i].ymax);
         rayState.final_bb[3] = cv::Point2f(bboxes.list[i].xmax, bboxes.list[i].ymax);
         for (int c=0; c<4;c++)
            rayState.w_corner[c]= (Eigen::Vector3d(0.0, 0.0, 0.0));
         rayState.lock_proba = 0;
         rayState.locked_def= false;
         int width = bboxes.list[i].xmax-bboxes.list[i].xmin;
         int lenght = bboxes.list[i].ymax-bboxes.list[i].ymin;
         rayState.class_ = bboxes.list[i].Class;
         rayState.w_l = std::make_pair(width,  lenght);



         //rayState.w_corner= {corner_init,corner_init,corner_init,corner_init};
          added++;

         //rayState.pixel.push_back(boundingBox);
         //add the firstpose and the pixel values in the sliding window
         bbTracker_.bbox_State_vect.push_back(rayState); //we push the

}
}
               cout << "After adding" <<  added<<", now we have" <<bbTracker_.bbox_State_vect.size() << "bounding boxes ";

//Dealing with unmatched tracker
                    for (int j=0; j<bbTracker_.bbox_State_vect.size(); j++)
                    {

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
            //bbTracker_.reproj(0.4);
            if (abs(tmp_V[0])<0.2 ||abs(tmp_V[1])<0.2 ||abs(tmp_V[2])<0.2 &&  bbTracker_.bbox_State_vect.size()>1 ){
            bbTracker_.nonMaxSupp();
            }
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
   // ros::Subscriber sub_cam_image = n.subscribe("/hummingbird/vi_sensor/left/image_raw", 2000, img_callback);
    ros::Subscriber sub_cam_image = n.subscribe("/cam0/image_raw", 2000, img_callback);
    pub_corner_point = n.advertise<sensor_msgs::PointCloud>("corner_point", 1000);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    //ros::Subscriber bb_sub_= n.subscribe("bounding_boxes", 2000, boundingboxesCallback);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("lines", 2000);
    cube_pub = n.advertise<visualization_msgs::MarkerArray>("cube_bbox", 2000);
    temp_cube_pub = n.advertise<visualization_msgs::MarkerArray>("temp_cube_bbox", 2000);
    //track_image_pub_ = it.advertise("track_overlay_image", 1);
    track_image_pub_ = n.advertise<sensor_msgs::Image>("track_overlay_image",2000); //for the bbox check
    optical_image_pub_ = n.advertise<sensor_msgs::Image>("optical_result",2000); //for the bbox check

    message_filters::Subscriber<sensor_msgs::Image> sub_darknet_image(n, "/detection_image", 2000);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub_(n,"/bounding_boxes", 2000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_darknet_image, bb_sub_);
     sync.registerCallback(boost::bind(&sync_callback, _1, _2));


//    message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes>
//            sync(sub_darknet_image, bb_sub_, 2000);
//    sync.registerCallback(boost::bind(&sync_callback, _1, _2));

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
