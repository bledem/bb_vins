#include <opencv2/calib3d.hpp>
#include "tracker.h"
#include <set>
using namespace std;

//TO DO match with inlier INSIDE the box in checkRedundancy
//TO DO undistort the vector we use for calculation

// void bbTracker_t::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
// {
//   ROS_DEBUG(" USB image received.");

//   cv_bridge::CvImagePtr cam_image;

//   try {
//     cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   } catch (cv_bridge::Exception& e) {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
// return;
//   }
//   prev_frame= cur_frame;
//   cur_frame = cam_image->image; //associated with the bounding box
//   ROS_INFO_STREAM("Has got a new darknet_ros detected image~~~~~~~~~~~~~~~~~~~~~~~~~~" );


//  // cv::namedWindow("detections YOLO3", cv::WINDOW_AUTOSIZE);
//  // cv::imshow("detections YOLO3",cam_image->image );
//   //cv::waitKey(0);


// }


 boundingBoxState_t::boundingBoxState_t(){}


bool bbTracker_t::IOU(Utility::bbox<float> bboxes1, Utility::bbox<float> bboxes2){
    float xx1, xx2, yy1,yy2, w,h, inter_area, o;

 xx1 = std::max(bboxes1.xmin, bboxes2.xmin); //tl
 yy1 = std::max(bboxes1.ymin, bboxes2.ymin);
 xx2 = std::min(bboxes1.xmax, bboxes2.xmax); //br
 yy2 = std::min(bboxes1.ymax, bboxes2.ymax);
 w = std::max(float(0.0), xx2 - xx1);
 h = std::max(float(0.0), yy2 - yy1);
 inter_area = w*h; //inter area
 float bb1_area = (bboxes1.xmax-bboxes1.xmin)*(bboxes1.ymax - bboxes1.ymin);
 float bb2_area = (bboxes2.xmax - bboxes2.xmin)*(bboxes2.ymax - bboxes2.ymin );
 o = inter_area / ((bb1_area + bb2_area)- inter_area);
 //cout << "IOU result" << o << "thresh" << thresh << endl;
  if (o>thresh){
         return true;
}else{
         return false;
}
}

//push in transform tar_bbox in undisort_bbox
Utility::bbox<float> bbTracker_t::undistortPoint(Utility::bbox<float> tar_bbox){

    int cx =363.0;
    int cy=248.1 ;
    int fx=461.6;
    int fy=460.3;
    //int K_arr[] = {fx, 0,cx, 0, fy, cy, 0, 0, 1};
    //cv::Mat K(3, 3, CV_8UC(1), K_arr);
    cv::Mat K = (cv::Mat_<float>(3,3) << fx, 0,cx, 0, fy, cy, 0, 0, 1);

    //float ditort[4] = {-0.2917,0.08228,0.00005333,-0.0001578};
    cv::Mat distort_coeff = (cv::Mat_<float>(1,4) << -0.2917,0.08228,0.00005333,-0.0001578);
    //cout << "kalibr and distort" << K << " " << distort_coeff << endl;
    //for (unsigned k=0; k<bbox_State_vect.size(); k++) { //to optimize
    std::vector<cv::Point2f> outputUndistortedPoints, input;

        cv::Point2f bbox_corner[4]; //tl, tr, bl, br
        bbox_to_points_cv(tar_bbox, bbox_corner);
        for (int i=0; i<4; i++){
            input.push_back(bbox_corner[i]);
        }
        cv::undistortPoints(input, outputUndistortedPoints, K, distort_coeff, cv::Mat());

        //bbox_corner[i] = un_pt;

        Utility::bbox<float> un_bbox;
        un_bbox.xmin = input[0].x ;
        un_bbox.xmax = input[4].x;
        un_bbox.ymin = input[0].y;
        un_bbox.ymax = input[4].y;
        //bbox_State_vect[k].un_pixel.push_back(un_bbox);
        return un_bbox;

    }


//cv::undistort() is an approximate iterative algorithm that estimates the normalized original point coordinates
//out of the normalized distorted point coordinates (“normalized” means that the coordinates do not depend
//on the camera matrix).
void bbTracker_t::project_pixel_to_pixel(){

    for (unsigned k=0; k<bbox_State_vect.size(); k++) {
        Eigen::Vector3d first_bbox[4];
        size_t nb_pose= bbox_State_vect[k].rotations_vec.size();
        bbox_to_points_eigen(bbox_State_vect[k].un_pixel[0],first_bbox );

        for (unsigned int i=0; i<4; i++){
            if (bbox_State_vect[k].depth[i]>0){

 //cout << "for the bbox id" <<bbox_State_vect[k].bbox_id <<"in project pixel to pixel first_saved bbox point" <<  first_bbox[i].transpose() << "bbox_State_vect["<<k<<"].depth["<<i<<"]=" << bbox_State_vect[k].depth[i] << endl;
    Eigen::Vector3d pts_camera_i = first_bbox[i] / bbox_State_vect[k].depth[i];
    Eigen::Vector3d pts_imu_i = R_imu_cam_ * pts_camera_i + p_C_I_;
    Eigen::Vector3d pts_w = bbox_State_vect[k].rotations_vec[0] * pts_imu_i + bbox_State_vect[k].poses_vec[0];

    if (k==0){
        bbox_State_vect[k].p_f_G_tl = pts_w;
    }else if (k == 3){
        bbox_State_vect[k].p_f_G_br = pts_w;
    }

    Eigen::Vector3d pts_imu_j = bbox_State_vect[k].rotations_vec[nb_pose].inverse() * (pts_w - bbox_State_vect[k].poses_vec[nb_pose]);
    Eigen::Vector3d pts_camera_j = R_imu_cam_.inverse() * (pts_imu_j - p_C_I_);
    pts_camera_j = pts_camera_j / pts_camera_j.z();
    bbox_State_vect[k].deduced_pixel[i]= cv::Point2f ((pts_camera_j.x()*f_u)+c_u, (pts_camera_j.y()*f_v)+c_v);
//    Eigen::Map<Eigen::Vector2d> residual(residuals);
                                            }
                                        }
                                }
                            }


void bbTracker_t::project_pixel_to_world(  Utility::ray<float>& tl_ray, Utility::ray<float>& br_ray,
                             Utility::bbox<float> &detected_bbox ){
    Eigen::Vector3d tl_ray_cam, br_ray_cam;

    //in CAM frame
     float x1  = ((detected_bbox.xmin-c_u)*z_C )/f_u;
     float y1 = ((detected_bbox.ymin-c_v)*z_C )/f_v;
     //cout <<"from imu position=" <<curr_imu_state_.p_I_G<< endl;

     //cout <<"from bbox xmin=" << detected_bbox.xmin<< ", ymin= " <<detected_bbox.ymin << endl;
     //cout <<"from bbox xmax=" << detected_bbox.xmax<< ", ymax= " <<detected_bbox.ymax << endl;

     //cout <<"from infos " << camera_.c_u << "," <<bb_state_.img_w_ << ", " << camera_.f_u <<endl;

     tl_ray_cam << x1, y1, z_C;
    //from VINS
    Eigen::Quaterniond qic{R_imu_cam_};
    qic.normalize();

//    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * tl_ray_cam + p_C_I_;
    Eigen::Vector3d pts_w = curr_imu_state_.q_IG * pts_imu_i + curr_imu_state_.p_I_G;
    //in WORLD frame
   tl_ray.p_GR= pts_w;
           //curr_imu_state_.p_I_G + curr_imu_state_.q_IG.inverse()*tl_ray_cam;
   // cout << "tl_ray in world frame" << tl_ray.p_GR << endl;



    float x2  = (detected_bbox.xmax-c_u)*z_C/f_u;
    float y2 = (detected_bbox.ymax-c_v)*z_C/f_v;
    br_ray_cam << x2, y2, z_C;

    pts_imu_i = qic * tl_ray_cam + p_C_I_;
    pts_w = curr_imu_state_.q_IG * pts_imu_i + curr_imu_state_.p_I_G;
    br_ray_cam = p_C_I_ + R_imu_cam_ * tl_ray_cam;
    br_ray.p_GR= pts_w;


  //cout << "br_ray in world frame" << br_ray.p_GR << endl;

}

//void bbTracker_t::checkRedundancy(){
//    for (unsigned k=0; k<bbox_State_vect.size(); k++) {

//    }
//}

void bbTracker_t::project_world_to_pixel(  Eigen::Vector3d corner[],
                             Utility::bbox<float> &predicted_bbox ){
     Eigen::Vector3d tl_ray_cam_in_C, br_ray_cam_in_C;
    Eigen::Quaterniond qci{R_imu_cam_.inverse()};
    Eigen::Quaterniond q_CG = qci * curr_imu_state_.q_IG;
    q_CG.normalize();

      //tl_ray_cam_in_C = q_CG.toRotationMatrix()*bbox_state.r_tl.p_GR;

      tl_ray_cam_in_C = curr_imu_state_.q_IG.inverse() *(corner[0] - curr_imu_state_.p_I_G);
      tl_ray_cam_in_C = R_imu_cam_.inverse()* (tl_ray_cam_in_C- p_C_I_) ; // paranthesis or not ?
      tl_ray_cam_in_C /= tl_ray_cam_in_C.z();
      //cout << "tl in cam frame" << tl_ray_cam_in_C << endl;
      predicted_bbox.xmin = (tl_ray_cam_in_C.x()*f_u)+c_u;
      predicted_bbox.ymin = (tl_ray_cam_in_C.y()*f_v)+c_v;

      //br_ray_cam_in_C = q_CG.toRotationMatrix()*bbox_state.r_br.p_GR;
      //br_ray_cam_in_C /= br_ray_cam_in_C.z();
      br_ray_cam_in_C = curr_imu_state_.q_IG.inverse() *(corner[3]- curr_imu_state_.p_I_G);
      br_ray_cam_in_C = R_imu_cam_.inverse()* (br_ray_cam_in_C- p_C_I_ );
      //cout << "br in cam frame" << br_ray_cam_in_C << endl;

      predicted_bbox.xmax  = (br_ray_cam_in_C.x()*f_u/br_ray_cam_in_C.z())+c_u;
      predicted_bbox.ymax = (br_ray_cam_in_C.y()*f_v/br_ray_cam_in_C.z())+c_v;


}

//we lock the box if more than three point
void bbTracker_t::lock_bbox(){

    //to draw the rectangle
    std::array<float,3> avg= {0.0};
    for (unsigned k=0; k<bbox_State_vect.size(); k++) {

        int count=0;
        for (unsigned int i=0; i<4; i++){
            if (bbox_State_vect[k].w_corner[i][0] != 0.0){ //if the point exists
                avg[0]+= bbox_State_vect[k].w_corner[i][0];
                avg[1]+= bbox_State_vect[k].w_corner[i][1];
                avg[2]+= bbox_State_vect[k].w_corner[i][2];
                count++;
            }

        }
        if (count==4){
            for (unsigned int i=0; i<4; i++){
                    bbox_State_vect[k].locked_corner[i] = bbox_State_vect[k].w_corner[i];
                    bbox_State_vect[k].lock = true;
        }
              bbox_State_vect[k].avg = avg;
    }else if (count ==3){
             bbox_State_vect[k].avg = avg;
        }

    }
}


void bbTracker_t::shift_bbox(Utility::bboxState<float>& bbox_state){
    std::vector<cv::Point2f> corners, corners_forw; // will contain all the bbox tl and br of the images to be shifted
    vector<uchar> status;
    vector<float> err;
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymin));
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymax));
//        corners.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymax));
//        corners.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymin));

cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, corners, corners_forw, status, err, cv::Size(21, 21), 3);
bbox_state.prev_detection =  bbox_state.cur_detection;

bbox_state.cur_detection.xmin = corners_forw[0].x;
bbox_state.cur_detection.ymin = corners_forw[0].y;
bbox_state.cur_detection.xmax = corners_forw[1].x;
bbox_state.cur_detection.ymax = corners_forw[1].y;

}



void bbTracker_t::shift_frame(cv::Mat cur_frame){ //shift the bbox with coming frame
    if(prev_frame.empty()){
        prev_frame=cur_frame;
        return;
    }
for (auto bbox_state : bbox_State_vect){

    shift_bbox(bbox_state);
}
prev_frame=cur_frame;
}

void bbTracker_t::bbox_to_points_eigen(Utility::bbox<float> bbox, Eigen::Vector3d (&bbox_corner)[4]){
    Eigen::Vector3d  tl, tr, bl,br;
    tl << bbox.xmin, bbox.ymin, 1;
    tr << bbox.xmax, bbox.ymin, 1;
    bl << bbox.xmin, bbox.ymax, 1;
    br << bbox.xmax, bbox.ymax, 1;
    bbox_corner[0]=tl;
    bbox_corner[1]=tr;
    bbox_corner[2]=bl;
    bbox_corner[3]=br;
}
void bbTracker_t::bbox_to_points_cv(Utility::bbox<float> bbox, cv::Point2f (&bbox_corner)[4]){
    cv::Point2f tl, br, tr, bl;
    tl = cv::Point2f(bbox.xmin, bbox.ymin);
    br = cv::Point2f(bbox.xmax, bbox.ymax);
    tr = cv::Point2f(bbox.xmax, bbox.ymin);
    bl = cv::Point2f(bbox.xmin, bbox.ymax);
    bbox_corner[0]=tl;
    bbox_corner[1]=tr;
    bbox_corner[2]=bl;
    bbox_corner[3]=br;
}


//void bbTracker_t::update_feature_pose();
//bbox_State_vect[k].feature_id[i] contains the id we have to take in feature attribute of estimator which is a list of FeaturePerId having the attribute feature_id and estimated_depth

void bbTracker_t::update_id(const std::map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image){
   double time_diff=0.0;
   float min_distance[4]; //tl, tr, bl, br
   float dist;
   size_t nb_feature = image.size();
   bbox_feature_id.clear();

   //for all the detected boxes
   int box_nb=0;

   if (bbox_State_vect.size()>0){
       time_diff = ros::Time::now().toSec()- bbox_State_vect[0].time;
       //cout <<"time difference between last frame and last detection" << std::setprecision(10) <<ros::Time::now().toSec() <<"  " <<bbox_State_vect[0].time << " " << time_diff  << "ms" << endl;
   }
    for (unsigned k=0; k<bbox_State_vect.size(); k++) {


       // if (bboxe_state.age <= 1 ) { //if it is realtively new frame and the pixel would match we try to match with a feature
       //     bboxe_state.age ++;

        //Create a list of the corner of this box
            Eigen::Vector3d bbox_corner[4];
            size_t id[4] = {0,0,0,0};

            bbox_to_points_eigen(bbox_State_vect[k].cur_detection,bbox_corner );
            //cout << "++++++++++++tl" << tl << "\n tr" << tr << "\n bl" << bl << "\n br" << br << endl;
            //for each corner

            // image is constructed by : image[feature_id].emplace_back(camera_id,  xyz_uv_velocity); id.first = camid , id.secund = xyz_uv_velocity
            // int feature_id = id_pts.first;
            bool renew = true ;
            for(int i=0; i<4; i++){ //tl, tr, bl, br
                        if ( (bbox_State_vect[k].feature_id[i] ==0 && (time_diff < 50 || bbox_State_vect[k].age<2)) || renew ) {
                            //if the corner was not found before or it a new detection we look for appropriate feature
                        //init the min distance
                        min_distance[i] = thresh_pixel+1;
                        //in all the detected features we find the closest one for this corner
                        for (auto &id_pts : image){ //for all the points of one feature
                                Eigen::Vector3d pix_pts ((id_pts.second[0].second.head<5>())[3],(id_pts.second[0].second.head<5>())[4], 1);
                                //cout << "DEBUG ~~ (x,y un) img_pts are" << id_pts.second[0].second.head<5>() <<"and pu pv "<< (id_pts.second[0].second.head<5>())[3]   <<" , feature id " << id_pts.first << ",  nb_feature second" << id_pts.second[nb_feature].second.head<2>() << endl;
                                //cout <<"DEBUG" << pix_pts.transpose() << "bbox corner" << bbox_corner[0].transpose() << " and" << bbox_corner[4].transpose() << endl;

                                bool in_x = (pix_pts[0]>bbox_corner[0][0] && pix_pts[0]<bbox_corner[3][0] ); //feature inside the box in x
                                bool in_y = (pix_pts[1]>bbox_corner[0][1] && pix_pts[1]<bbox_corner[3][1] ); //feature inside the box in x
                                dist = (bbox_corner[i]-pix_pts).norm();
                                //std::cout << "dist" << dist << "difference of" << bbox_corner[i] << " and " <<pix_pts << std::endl;
//                                if (in_x ){
//                                        cout <<"XRANGE" << pix_pts.transpose() << "is in x bbox" << endl;
//                                    }
//                                    if (in_y ){
//                                            cout <<"YRANGE" << pix_pts.transpose() << "is in y bbox" << endl;
//                                        }
                                if (dist< min_distance[i] && in_x && in_y ){
                                    min_distance[i]= dist;
                                    //std::cout << "in update id pixel observed diff=" << pix_pts-bbox_corner[i] << "pix_pts" << pix_pts.transpose()<<  "with feature id" << id_pts.first << std::endl;
                                    id[i] = id_pts.first;
                                }
                    }
        //end of checking all the detected points for this image
                    //4 values for each box
                            //update the id of the closest feature of bboxState
                    if (min_distance[i]<thresh_pixel && id[i]!=0 && (id[i] !=bbox_State_vect[k].feature_id[i])  ){
                        //cout<< "DEBUG found feature id" <<id[i] << "bboxstate id"<<bbox_State_vect[k].feature_id[i] << endl;

                    bbox_State_vect[k].feature_id[i] = id[i]; // otherwise stay 0

                    std::cout <<std::setprecision(10) <<" in update_id called in process/ Compare feature and bbox the box id" << bbox_State_vect[k].bbox_id
                             << " and numero  "<< box_nb<<" in the bboxState_vec, for corner: " << i << " min dist is: " << min_distance[i]
                             << " we give feature id is:" << id[i] << std::endl;
                     }else if (bbox_State_vect[k].feature_id[i] !=0) { //if the corner was already tracked

                        //check if the feature id is not too old
                         }else{
                         //we consider 0 as a fail in finding a close feature

//                        }else{
//                            auto id_tl = find(id_to_delete.begin(), id_to_delete.end(),(bbox_State_vect[k].feature_id[i]));
//                            if (id_tl != id_to_delete.end()){ //if the id contained by this bounding box has been deleted
//                                bbox_State_vect[k].feature_id[i]=0;
//                                cout << "Erase discarded id" << endl;
                            }


                        } //end of for loop on corner not already tracked HERE

                       // bbox_feature_id.push_back(bbox_State_vect[k].feature_id[i]);
            } //end if they wee new feature
           // end of loop on corners
                       // } //end if the detection is from this loop or the previous one


        box_nb++;
    } //

    //return bbox_feature_id;

}



void bbTracker_t::update_pose( Eigen::Vector3d curr_pos_p,
                       Eigen::Quaterniond curr_pos_q,  Eigen::Matrix3d R_imu_cam, Eigen::Vector3d p_C_I //init only once
){
    if (!init_){
        cout << "init of bbTracker done" << endl;
    world_orig_p_ = curr_pos_p;
    world_orig_q_ = curr_pos_q;
   // auto test = camera_->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
init_=true;
    }
    //TO DO vect treatment
        curr_imu_state_.p_I_G = curr_pos_p ;
        curr_imu_state_.q_IG = curr_pos_q ;
        R_imu_cam_ =R_imu_cam;
        p_C_I_ = p_C_I;

        if (frame_count > max_age_frame){
            bbox_State_vect.clear();
            frame_count=0;
            bb_state_.img_bboxes.list.clear();
            //std::cout << "drop the last detection" << endl;
        }

        for (unsigned i=0; i< bbox_State_vect.size(); i++) {
           // cout <<"in process, age is " << bbox_State_vect[i].age << ". "<< endl;
            if(bbox_State_vect[i].age >max_age_detection){
                bbox_State_vect.erase(bbox_State_vect.begin()+i);
                cout << "-------we erase" << i << "from bbox state" << endl;
            }
        }
            //std::cout << "the predicted bbox is " << frame_count <<"frame year old and contains"<< bbox_State_vect.size() << "boxes" << endl;


}




bbTracker_t::bbTracker_t()
{

 init_=false;
 z_C=1;
 bbox_State_vect.clear();
 thresh = 0.6;
 thresh_pixel = 30;
 max_age_detection=3; //increase every time a detection occur without have been updated  min_hit=0;
 max_age_frame =5;
 frame_count=0;
 id_count=0;
  c_u=363.0;
  c_v=248.1 ;

  f_u=461.6;
  f_v=460.3;
}






