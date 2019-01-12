#ifndef Utility_ROS_TRACKER_H_
#define Utility_ROS_TRACKER_H_

#pragma once

// Standard
#include <vector>
#include <map>
#include <unordered_map>
#include <stdlib.h>
#include <math.h>
#include <iostream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"

// Fast corner detection
#include "utility/utility.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <opencv2/core/eigen.hpp>





// Eigen
#include <Eigen/Dense>

// Matrix utils & types



typedef cv::Point2f Point2f; 
typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<size_t> IdVector;
//typedef std::vector<Utility::Vector2<float>,
//                    Eigen::aligned_allocator<Utility::Vector2<float>>>
//                      OutFeatureVector;




class boundingBoxState_t{
public:
    boundingBoxState_t();

   // std::vector<Utility::imgBboxes<float>> img_bboxes_states_;
    Utility::imgBboxes<float> img_bboxes;
    cv::Mat img_raw;
    float img_w_, img_h_;

   // Utility::bbox<float> one_bbox;
   // void cameraCallback(const sensor_msgs::ImageConstPtr &msg);

   // ros::Subscriber cam_sub = nh_.subscribe("detections", 200, &boundingBoxState_t::cameraCallback, this);

    //    message_filters::Subscriber<boundingBoxState_t> bb_sub(nh, "bounding_boxes", 1);
    //    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image_bbox", 1);

};


class bbTracker_t
{
public:

    bbTracker_t( );


//never change
    Eigen::Quaterniond world_orig_q_;
    Eigen::Vector3d world_orig_p_;
//update with time
    Utility::imuState<float> curr_imu_state_;
    Eigen::Matrix3d R_imu_cam_;
    camodocal::CameraPtr camera_;
    float z_C;
    float thresh, thresh_pixel; //for pixel distance
    size_t id_count;



    cv::Mat prev_frame, cur_frame;
    cv::Mat projection_param;
    int max_age_frame, max_age_detection, min_hit, frame_count;
    Utility::imgBboxes<float> last_detected_bbox;
    Utility::imgBboxes<float> new_predicted_bbox;
     Eigen::Vector3d p_C_I_;
    std::vector<Utility::bboxState<float>> bbox_State_vect;
    std::vector< cv::Mat> frame_queue;
    std::vector< Utility::imgBboxes<float>> detected_bboxes_queue;
    boundingBoxState_t bb_state_;
    bool init_;

    void init(  Eigen::Vector3d curr_pos_p,
                Eigen::Quaterniond curr_pos_q,  Eigen::Matrix3d R_imu_cam, Eigen::Vector3d p_C_I);
   void update_pose(Eigen::Vector3d curr_pos_p,
                    Eigen::Quaterniond curr_pos_q, Eigen::Matrix3d R_imu_cam, Eigen::Vector3d p_C_I, int nb_frame);
   void newBB(Utility::imgBboxes<float> img_bboxes);
   void project_pixel_to_world(  Utility::ray<float>& tl_ray, Utility::ray<float>& br_ray,
                                Utility::bbox<float> &detected_bbox );
   void project_world_to_pixel(  Utility::bboxState<float> bbox_state,
                                Utility::bbox<float> &predicted_bbox );
   //corner_detector::IdVector find_feature( std::vector<Utility::Vector2<float>,
                                           //Eigen::aligned_allocator<Utility::Vector2<float>>>  new_features_dist_, corner_detector::IdVector new_ids, std::vector<size_t> id_to_delete);

   bool IOU(Utility::bbox<float> bboxes1, Utility::bbox<float> bboxes2);
   void cameraCallback(const sensor_msgs::ImageConstPtr &msg);


};





#endif
