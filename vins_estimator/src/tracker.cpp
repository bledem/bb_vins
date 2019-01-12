#include <opencv2/calib3d.hpp>
#include "tracker.h"
#include <set>
using namespace std;



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

//void bbTracker_t::boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes &bboxes_ros){
//    //double img_bboxes_time = bboxes_ros.header.stamp.toSec();
//    double img_bboxes_time = ros::Time::now().toSec();
//    bb_state_.img_w_= bboxes_ros.img_w;
//    bb_state_.img_h_= bboxes_ros.img_h;
//    //cout << bboxes_ros.img_w < " is saved in " << bb_state_.img_w_ << endl;
//    Utility::imgBboxes<float> bboxes;
//    for (unsigned int i=0; i< bboxes_ros.bounding_boxes.size(); i++){
//        Utility::bbox<float> boundingBox;
//        try{
//        boundingBox.Class = bboxes_ros.bounding_boxes.at(i).Class ;
//        boundingBox.prob  = bboxes_ros.bounding_boxes.at(i).probability ;
//        boundingBox.xmin  = bboxes_ros.bounding_boxes.at(i).xmin ;
//        boundingBox.ymin = bboxes_ros.bounding_boxes.at(i).ymin ;
//        boundingBox.xmax  = bboxes_ros.bounding_boxes.at(i).xmax ;
//        boundingBox.ymax  = bboxes_ros.bounding_boxes.at(i).ymax ;
//        bboxes.list.push_back(boundingBox);
//          //ROS_INFO_STREAM("Size of bounding box " << boundingBox.xmin <<","<< boundingBox.ymin );
//        }catch(const std::exception& e){
//            cout << "Wrong bounding box format, skipped" << endl;
//        }

//    }
//    if (bboxes_ros.bounding_boxes.size() > 0){
//        ROS_INFO_STREAM("Has " << bboxes_ros.bounding_boxes.size() << " bounding boxes detected and had already " <<bbox_State_vect.size() << "~~~~~~~~~~~~~~~~~~~~~~~~~~" );

//    }
//    bb_state_.img_raw = cur_frame;
//    bb_state_.img_bboxes= bboxes;
//    bb_state_.img_bboxes.time =img_bboxes_time;
//   // img_bboxes_states_.push_back(img_bboxes);
//    frame_count =0;

    //bbox_State_vect.clear();

    //for every bounding box of our state
//std::vector<Utility::bboxState<float>> reserve;

//      for (int i=0; i<bboxes.list.size(); i++){
//            bool tracked;
//          //check if bbox is already tracked
//          for (auto bbox_state : bbox_State_vect){
//              //cout << "for bbox no" << bbox_state.bbox_id << endl;
//    if(IOU(bboxes.list[i], bbox_state.cur_detection)){
//                    bbox_state.prev_detection = bbox_state.cur_detection; //update already existing bbox
//                    bbox_state.cur_detection = bboxes.list[i];
//                    bbox_state.age=0;
//                    bbox_state.nb_detected++;
//                    bbox_state.time = img_bboxes_time;
//                    bbox_state.associated = true;
//                    tracked=true;
//                    cout << "tracked !! bbox_id is "<<bbox_state.bbox_id <<" tracked for:"<< bbox_state.nb_detected<<" detection ,and corner id are:" << bbox_state.feature_id[0] << ", " << bbox_state.feature_id[1] << ", " << bbox_state.feature_id[2] <<
//                            ", " <<bbox_state.feature_id[3] << endl;
//                    break; //quit the comparaison with the previous bbox updated
//          } else {
//               if (bbox_state.time /= bboxes_ros.header.stamp.toSec()){ //update only once
//               bbox_state.age++; //case the bbox was not matching we add up once its age if not done
//               bbox_state.prev_detection = bbox_state.cur_detection ;
//               bbox_state.associated = false;
//               bbox_state.nb_detected =0;
//               bbox_state.time = img_bboxes_time;
//                tracked=false;
//               }
//           }
//           } //we dealt with all the previous saved bbox

//           if(tracked==false || bbox_State_vect.size() == 0){ //

//           //Box never has been tracked we will augment the bboxState
//           Utility::bboxState<float> rayState; //create a bbox state
//           project_pixel_to_world(rayState.r_tl ,rayState.r_br , bboxes.list[i] ); //init the ray
//           rayState.bbox_id = id_count;
//           rayState.cur_detection = bboxes.list[i];
//           rayState.age=0;
//           rayState.nb_detected=0;
//           rayState.time= img_bboxes_time;
//           id_count++;
//           rayState.associated = true;
//          // tl_G.p_GR = curr_imu_state_.p_I_G +  R_imu_cam_*(tl_C.p_GR - (-1*p_C_I_)) ;
//          // br_G.p_GR = curr_imu_state_.p_I_G +  R_imu_cam_*(br_C.p_GR - (-1*p_C_I_)) ;

//           reserve.push_back(rayState);
//}

//          }


//      for (auto bbox_added : reserve){
//          bbox_State_vect.push_back(bbox_added);
//      }
//      cout << "After adding" <<reserve.size() <<", now we have" <<bbox_State_vect.size() << endl;
//      reserve.clear();

//}


void bbTracker_t::project_pixel_to_world(  Utility::ray<float>& tl_ray, Utility::ray<float>& br_ray,
                             Utility::bbox<float> &detected_bbox ){
    Eigen::Vector3d tl_ray_cam, br_ray_cam;

    float c_u=363.0;
    float c_v=248.1 ;

    float f_u=461.6;
    float f_v=460.3;


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
//    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * tl_ray_cam + p_C_I_;
    Eigen::Vector3d pts_w = curr_imu_state_.q_IG * pts_imu_i + curr_imu_state_.p_I_G;

    //cout << "tl in cam frame" << tl_ray_cam << endl;
    //in IMU frame
    tl_ray_cam = p_C_I_ + R_imu_cam_ * tl_ray_cam;
    //cout << "tl in imu frame" << tl_ray_cam << endl;
    //in WORLD frame
   tl_ray.p_GR= pts_w;
           //curr_imu_state_.p_I_G + curr_imu_state_.q_IG.inverse()*tl_ray_cam;
    cout << "tl_ray in world frame" << tl_ray.p_GR << endl;



    float x2  = (detected_bbox.xmax-c_u)*z_C/f_u;
    float y2 = (detected_bbox.ymax-c_v)*z_C/f_v;
   // br_ray_cam << x2, y2, z_C;
    //cout << "br in cam frame" << br_ray_cam << endl;
   // br_ray_cam = p_C_I_ +R_imu_cam_ * br_ray_cam;
    //cout << "br in imu frame" << br_ray_cam << endl;
    //in WORLD frame
     //br_ray.p_GR= curr_imu_state_.p_I_G + curr_imu_state_.q_IG.inverse()*br_ray_cam; // transpose or not??

  //cout << "br_ray in world frame" << br_ray.p_GR << endl;
//  cout <<"from x,y " << detected_bbox.xmax<< "," <<detected_bbox.ymax << endl;

}



//void bbTracker_t::project_world_to_pixel(  Utility::bboxState<float> bbox_state,
//                             Utility::bbox<float> &predicted_bbox ){
//    Utility::Vector3<float > tl_ray_cam_in_C, br_ray_cam_in_C;
//    Utility::Quaternion<float> q_CG = camera_.q_CI * curr_imu_state_.q_IG;
//    q_CG.normalize();

//      //tl_ray_cam_in_C = q_CG.toRotationMatrix()*bbox_state.r_tl.p_GR;

//      tl_ray_cam_in_C = curr_imu_state_.q_IG*(bbox_state.p_f_G_tl- curr_imu_state_.p_I_G);
//      tl_ray_cam_in_C = R_imu_cam_.inverse()* (tl_ray_cam_in_C- p_C_I_ );
//      //tl_ray_cam_in_C /= tl_ray_cam_in_C.z();
//      //cout << "tl in cam frame" << tl_ray_cam_in_C << endl;
//      predicted_bbox.xmin = (tl_ray_cam_in_C.x()*camera_.f_u/tl_ray_cam_in_C.z())+camera_.c_u;
//      predicted_bbox.ymin = (tl_ray_cam_in_C.y()*camera_.f_v/tl_ray_cam_in_C.z())+camera_.c_v;

//      //br_ray_cam_in_C = q_CG.toRotationMatrix()*bbox_state.r_br.p_GR;
//      //br_ray_cam_in_C /= br_ray_cam_in_C.z();
//      br_ray_cam_in_C = curr_imu_state_.q_IG*(bbox_state.p_f_G_br- curr_imu_state_.p_I_G);
//      br_ray_cam_in_C = R_imu_cam_.inverse()* (br_ray_cam_in_C- p_C_I_ );
//      //cout << "br in cam frame" << br_ray_cam_in_C << endl;

//      predicted_bbox.xmax  = (br_ray_cam_in_C.x()*camera_.f_u/br_ray_cam_in_C.z())+camera_.c_u;
//      predicted_bbox.ymax = (br_ray_cam_in_C.y()*camera_.f_v/br_ray_cam_in_C.z())+camera_.c_v;


//}

//Take the new found feature and their id and return the id of these closest features
//corner_detector::IdVector bbTracker_t::find_feature( std::vector<Utility::Vector2<float>,
//                                                     Eigen::aligned_allocator<Utility::Vector2<float>>>  new_features_dist_, corner_detector::IdVector new_ids, std::vector<size_t> id_to_delete)
//{
//    double time_diff=0.0;
//   float min_distance[4]; //tl, tr, bl, br
//   float dist;
//   corner_detector::IdVector bbox_feature_id;

//   //for all the detected boxes
//   int box_nb=0;

//   if (bbox_State_vect.size()>0){
//       time_diff = ros::Time::now().toSec()- bbox_State_vect[0].time;
//       cout <<"time difference between last frame and last detection" << std::setprecision(10) <<ros::Time::now().toSec() <<"  " <<bbox_State_vect[0].time << " " << time_diff  << "ms" << endl;
//   }

//    for (int k=0; k<bbox_State_vect.size(); k++) {


//       // if (bboxe_state.age <= 1 ) { //if it is realtively new frame and the pixel would match we try to match with a feature
//       //     bboxe_state.age ++;

//        //Create a list of the corner of this box
//            Utility::Vector2<float> bbox_corner[3];
//            size_t id[4] = {0,0,0,0};
//            Utility::Vector2<float>  tl, tr, bl,br;
//            tl << bbox_State_vect[k].cur_detection.xmin, bbox_State_vect[k].cur_detection.ymin;
//            tr << bbox_State_vect[k].cur_detection.xmax, bbox_State_vect[k].cur_detection.ymin;
//            bl << bbox_State_vect[k].cur_detection.xmin, bbox_State_vect[k].cur_detection.ymax;
//            br << bbox_State_vect[k].cur_detection.xmax, bbox_State_vect[k].cur_detection.ymax;
//            bbox_corner[0]=tl;
//            bbox_corner[1]=tr;
//            bbox_corner[2]=bl;
//            bbox_corner[3]=br;
//           // cout << "++++++++++++tl" << tl << "\n tr" << tr << "\n bl" << bl << "\n br" << br << endl;
//            //for each corner
//            if (new_features_dist_.size()>0){

//            for(int i=0; i<4; i++){
//                        if ( bbox_State_vect[k].feature_id[i] ==0 && (time_diff < 50 || bbox_State_vect[k].age<2))
//                        { //if the corner was not found before or it a new detection we look for appropriate feature
//                        //init the min distance
//                        min_distance[i] = (bbox_corner[i]-new_features_dist_[0]).norm();
//                        //in all the detected features we find the closest one for this corner
//                        for (int j=0; j<new_features_dist_.size(); j++)
//                        {
//                                dist = (bbox_corner[i]-new_features_dist_[j]).norm();
//                                //std::cout << "dist" << dist << "difference of" << bbox_corner[i] << " and " <<new_features_dist_[j] << std::endl;
//                                if (dist< min_distance[i]){
//                                    min_distance[i]= dist;
//                                    //std::cout << "pixel observed diff" << new_features_dist_[j]-bbox_corner[i] << "with id" << new_ids[j] << std::endl;
//                                    id[i] = new_ids[j];
//                                }
//                    }
//        //
//                    //4 values for each box
//                    if (min_distance[i]<thresh_pixel && id[i]!=0){
//                    bbox_feature_id.push_back(id[i]);
//                    bbox_State_vect[k].feature_id[i] = id[i]; // otherwise stay 0

//                    std::cout <<std::setprecision(10) <<"In bbTRacker the min_distance<thresh_pixel of the box" << bbox_State_vect[k].bbox_id << "placed in the vector nb"<< box_nb<<" for corner: " << i << " is: " << min_distance[i]
//                             << "id is:" << id[i] << std::endl;
//                     }else if (bbox_State_vect[k].feature_id[i] !=0) {


//                         }else {
//                         bbox_feature_id.push_back(0); //we consider 0 as a fail in finding a close feature
//                }
//                        }else{
//                            auto id_tl = find(id_to_delete.begin(), id_to_delete.end(),(bbox_State_vect[k].feature_id[i]));
//                            if (id_tl != id_to_delete.end()){ //if the id contained by this bounding box has been deleted
//                                bbox_State_vect[k].feature_id[i]=0;
//                                cout << "Erase discarded id" << endl;
//                            }

//                        }
//                        } //end of for loop on corner not already tracked

//                        } //end if they wee new feature
//           // end of loop on corners
//                       // } //end if the detection is from this loop or the previous one


//        box_nb++;
//    } //

//    return bbox_feature_id;

//}



void bbTracker_t::init( Eigen::Vector3d curr_pos_p,
                       Eigen::Quaterniond curr_pos_q,  Eigen::Matrix3d R_imu_cam, Eigen::Vector3d p_C_I
){
    if (!init_){
    world_orig_p_ = curr_pos_p;
    world_orig_q_ = curr_pos_q;
   // auto test = camera_->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);


    init_=true;
    }
}


void bbTracker_t::update_pose( Eigen::Vector3d curr_pos_p,
                               Eigen::Quaterniond curr_pos_q, Eigen::Matrix3d R_imu_cam, Eigen::Vector3d p_C_I, int nb_frame)
{

//TO DO vect treatment
    curr_imu_state_.p_I_G = curr_pos_p ;
    curr_imu_state_.q_IG = curr_pos_q ;
    R_imu_cam_ =R_imu_cam;
    p_C_I_ = p_C_I;

    frame_count+=nb_frame; //frame count is the nb of frame without any detection
    if (frame_count > max_age_frame){
        bbox_State_vect.clear();
        frame_count=0;
        //bb_state_.img_bboxes.list.clear();
        std::cout << "drop the last detection" << endl;
    }

    for (int i=0; i< bbox_State_vect.size(); i++) {
        if(bbox_State_vect[i].age >max_age_detection){
            bbox_State_vect.erase(bbox_State_vect.begin()+i);
            cout << "-------we erase" << i << "from bbox state" << endl;
        }
    }
        std::cout << "the predicted bbox is " << frame_count <<"frame year old and contains"<< bbox_State_vect.size() << "boxes" << endl;



}


bbTracker_t::bbTracker_t()
{

 init_=false;
 z_C=1;
 bbox_State_vect.clear();
 frame_count=0;
 thresh = 0.4;
 thresh_pixel = 30;
 max_age_detection=3; //increase every time a detection occur without have been updated  min_hit=0;
 max_age_frame =13;
 frame_count=0;
 id_count=0;
}






