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
#include <visualization_msgs/Marker.h>

#include "utility/utility.h"


#include "estimator.h"
#include "tracker.h"

#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;
bbTracker_t bbTracker_;
visualization_msgs::Marker line_list;
ros::Publisher marker_pub;


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
    //take the last values of estimator
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
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
    //cout << "in imu callbak" << endl;
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


    geometry_msgs::Point ray0;
            ray0.x = tmp_P[0];
            ray0.y = tmp_P[1];
            ray0.z = tmp_P[2];
//if (bb_sub_.getNumPublishers()>0 ){      //for (int i=0; i<ray_State_vect.size();i++){

   if (bbTracker_.bbox_State_vect.size()>0 ){
       line_list.type = visualization_msgs::Marker::LINE_STRIP;
       //line_list.id = 2;
       line_list.scale.x = 0.1;
       line_list.header.frame_id = "map";
       line_list.header.stamp = ros::Time::now();
      // line_list.ns = "my lines_" +bbTracker_.ray_State_vect.size().str();
       line_list.action = visualization_msgs::Marker::ADD;
       line_list.pose.orientation.w = 1.0;
      // line_list.points=[];


  // for (int i =0; i<bbTracker_.ray_State_vect.size(); i++){
int i=0;


        geometry_msgs::Point raytl, raybr;
        //cout << "We publish one bonding box out of" <<bbTracker_.bbox_State_vect.size() << endl;

        raytl.x =bbTracker_.bbox_State_vect[i].r_tl.p_GR[0];
        raytl.y =bbTracker_.bbox_State_vect[i].r_tl.p_GR[1];
        raytl.z =bbTracker_.bbox_State_vect[i].r_tl.p_GR[2];
//        raybr.x =bbTracker_.bbox_State_vect[i].r_br.p_GR[0];
//        raybr.y =bbTracker_.bbox_State_vect[i].r_br.p_GR[1];
//        raybr.z =bbTracker_.bbox_State_vect[i].r_br.p_GR[2];
        line_list.color.b = 1.0;
        line_list.color.r = 1.0;
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;
       // raytl.color.b = 1.0;
        line_list.points.push_back(ray0);

          line_list.points.push_back(raytl);
          line_list.points.push_back(ray0);

        //line_list.points.push_back(raybr);



       // cout << "r_tl" <<bbTracker_.bbox_State_vect[i].r_tl.p_GR << endl;
       // cout << "imu state " << imu_state.p_I_G << endl;

//            int lines_nb = 8;

//            if (line_list.points.size()> lines_nb){
//                cout << "lines number is"<<line_list.points.size() << endl;
//                line_list.points.clear();

       //   }

        marker_pub.publish(line_list);
        line_list.points.clear();


}
//}
else{
  cout << "no bounding box, delete the rays" << endl;
   line_list.action = visualization_msgs::Marker::DELETE;
   line_list.points.clear();
   marker_pub.publish(line_list);


}
}

void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes &bboxes_ros){
    //double img_bboxes_time = bboxes_ros.header.stamp.toSec();
    double img_bboxes_time = ros::Time::now().toSec();
    bbTracker_.bb_state_.img_w_= bboxes_ros.img_w;
    bbTracker_.bb_state_.img_h_= bboxes_ros.img_h;
    //cout << bboxes_ros.img_w < " is saved in " << bb_state_.img_w_ << endl;
    Utility::imgBboxes<float> bboxes;
    for (unsigned int i=0; i< bboxes_ros.bounding_boxes.size(); i++){
        Utility::bbox<float> boundingBox;
        try{
        boundingBox.Class = bboxes_ros.bounding_boxes.at(i).Class ;
        boundingBox.prob  = bboxes_ros.bounding_boxes.at(i).probability ;
        boundingBox.xmin  = bboxes_ros.bounding_boxes.at(i).xmin ;
        boundingBox.ymin = bboxes_ros.bounding_boxes.at(i).ymin ;
        boundingBox.xmax  = bboxes_ros.bounding_boxes.at(i).xmax ;
        boundingBox.ymax  = bboxes_ros.bounding_boxes.at(i).ymax ;
        bboxes.list.push_back(boundingBox);
          //ROS_INFO_STREAM("Size of bounding box " << boundingBox.xmin <<","<< boundingBox.ymin );
        }catch(const std::exception& e){
            cout << "Wrong bounding box format, skipped" << endl;
        }
        //bbTracker_.bb_state_.img_raw = cur_frame;
        bbTracker_.bb_state_.img_bboxes= bboxes;
        bbTracker_.bb_state_.img_bboxes.time =img_bboxes_time;
       // img_bboxes_states_.push_back(img_bboxes);
        bbTracker_.frame_count =0;
}
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
                    u_v_id.x() = relo_msg->points[i].x;
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
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
           // ROS_INFO("point size %u \n", img_msg->points.size());
            for (unsigned int i = 0; i < img_msg->points.size(); i++) //for all the points in the poincloud
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x; //undistorded
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i]; //pixel values
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);

                //std::cout << "v" << v << "feature_id" << feature_id << "camera_id" << "x and y " << x << "y" << y << "p_u" << p_u << "p_v" << p_v  << "velocity_x" << velocity_x << std::endl;

            }

            estimator.processImage(image, img_msg->header);

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
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)           
            update();
        bbTracker_.init(tmp_P, tmp_Q, estimator.ric[NUM_OF_CAM-1], estimator.tic[NUM_OF_CAM-1]); //check the value

        m_state.unlock();
        m_buf.unlock();
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
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

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    ros::Subscriber bb_sub_= n.subscribe("bounding_boxes", 2000, boundingboxesCallback);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
