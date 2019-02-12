#include <opencv2/calib3d.hpp>
#include "tracker.h"
#include <set>
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

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

string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};


 boundingBoxState_t::boundingBoxState_t(){}


bool bbTracker_t::IOU(Utility::bbox<float> bboxes1, Utility::bbox<float> bboxes2, float thresh_args){
    float xx1, xx2, yy1,yy2, w,h, inter_area;
    double o;
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
 //cout << std::setprecision(3) <<"IOU result" << o << "thresh" << thresh << endl;
  if (o>=thresh_args){
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


void bbTracker_t::predict_missing(Eigen::Vector3d (& w_corner)[4], int missing, float avg_x){
    switch (missing) {
    //cout << "in predict missing i value is" << missing << endl;

           case 0: //tl missing
        w_corner[missing][0]= avg_x;
        w_corner[missing][1]= w_corner[2][1];
        w_corner[missing][2]= w_corner[1][1];
        break;

           case 1: //tr missing
        w_corner[missing][0]= avg_x;
        w_corner[missing][1]= w_corner[3][1];
        w_corner[missing][2]= w_corner[0][1];
        break;

           case 2: //bl missing
        w_corner[missing][0]= avg_x;
        w_corner[missing][1]= w_corner[0][1];
        w_corner[missing][2]= w_corner[3][1];
        break;

           case 3: // br missing
        w_corner[missing][0]= avg_x;
        w_corner[missing][1]= w_corner[1][1];
        w_corner[missing][2]= w_corner[2][1];
           case 4:
        std::cout << "error in missing corner" << endl;
}
}


float calc_err(Eigen::Vector3d w_corner[4]){
    Eigen::Vector3d tl = w_corner[0];
    Eigen::Vector3d tr = w_corner[1];
    Eigen::Vector3d br = w_corner[3];
    Eigen::Vector3d bl = w_corner[2];

    float err_x = abs(tl[0] - tr[0]) + abs(tl[0]-br[0]) + abs(tr[0] - br[0]) + abs(br[0]-bl[0]);
    float err_y = abs(tr[1]-br[1]) + abs(tl[1]-bl[1]);
    float err_z = abs(bl[2]-br[2]) + abs(tl[1]-tr[2]);
    return err_x + err_y + err_z;
}


//we lock the box if more than three point
void bbTracker_t::lock_bbox(){
    cout<< "locked bbox "<<endl;

    //to draw the rectangle
    for (unsigned int k=0; k<bbox_State_vect.size(); k++) { //for each box
        std::array<float,3> avg= {0.0};
        std::vector<Eigen::Vector3d> vec_corner;
        Eigen::Vector3d avg_vec;
        size_t missing_ =4;
        size_t count_miss=0;
        int count=0;
        //cout << "k" <<k <<  "missing0 " << missing_  << endl;
        for (size_t i=0; i<4; i++){ //for each corner
            if (abs(bbox_State_vect[k].w_corner[i][0]) > 0.001 && abs(bbox_State_vect[k].w_corner[i][1]) > 0.001 && abs(bbox_State_vect[k].w_corner[i][2]) > 0.001 ){ //if the point exists
                avg[0]+= bbox_State_vect[k].w_corner[i][0]; //x
                avg[1]+= bbox_State_vect[k].w_corner[i][1]; //y
                avg[2]+= bbox_State_vect[k].w_corner[i][2]; //z
                count++;

            } else if (count_miss<=1) {
                missing_=i;
                //cout << "missing " << i <<" value is" << int(missing_) << endl;
                count_miss++;
            }
            }

        if (count == 3){ //we approximate the value of the missing corner
            predict_missing(bbox_State_vect[k].w_corner, missing_, avg[0]);
        }
        if ((count==4 || count ==3) ){
            //cout << "enough corner, count proba:" << bbox_State_vect[k].lock_proba+1 << " and id is " << bbox_State_vect[k].bbox_id << " with values " <<  bbox_State_vect[k].avg[0]<< " " << bbox_State_vect[k].avg[1] << " " << bbox_State_vect[k].avg[2] <<
            //        "locked size is  " <<  bbox_State_vect[k].locked_vec.size() <<endl;

             bbox_State_vect[k].lock_proba++;
         if (bbox_State_vect[k].lock_proba<15){
            for (unsigned int i=0; i<4; i++){
                    bbox_State_vect[k].locked_corner[i]=(bbox_State_vect[k].w_corner[i]);

        }
            bbox_State_vect[k].avg[0]=avg[0]/float(count);
            bbox_State_vect[k].avg[1]=avg[1]/float(count);
            bbox_State_vect[k].avg[2]=avg[2]/float(count);

            //cout <<"Count is" << count <<"with value" << bbox_State_vect[k].avg[0]<< " "<< bbox_State_vect[k].avg[1]<< " " << bbox_State_vect[k].avg[2] <<endl;
         } else if (bbox_State_vect[k].lock_proba >=15) {
             float err_temp = calc_err(bbox_State_vect[k].w_corner);
             float err_def = calc_err(bbox_State_vect[k].locked_corner_def);

             for (unsigned int i=0; i<4; i++){
                 if (bbox_State_vect[k].w_corner[i][0] != 0.0)
                     vec_corner.push_back(bbox_State_vect[k].w_corner[i]);
                 if (bbox_State_vect[k].lock_proba ==15 || err_def < err_temp)
                         bbox_State_vect[k].locked_corner_def[i] = bbox_State_vect[k].w_corner[i];
                         bbox_State_vect[k].locked_def= true;

                 if (i<3)
                    avg_vec[i]=avg[i]/float(count);


                 bbox_State_vect[k].locked_bbox= ( std::make_pair(vec_corner, avg_vec));
            //cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create();
//cout <<"before multritracker add" << endl;
//multiTracker->add(tracker, prev_frame, cv::Rect2d(bbox_State_vect[k].prev_detection.xmin,
//                    bbox_State_vect[k].prev_detection.ymin,
//                                (bbox_State_vect[k].prev_detection.xmax-bbox_State_vect[k].prev_detection.xmin),
//                                  (bbox_State_vect[k].prev_detection.ymax-bbox_State_vect[k].prev_detection.ymin)
                                //));
}


         }
         } else if (count<=2 && bbox_State_vect[k].lock_proba>1) {
             bbox_State_vect[k].lock_proba--;
             cout << "DECREASE THE LOCK to " <<  bbox_State_vect[k].lock_proba <<endl;
         }

//    }else if (count ==3){
//            for (unsigned int i=0; i<4; i++){

//                bbox_State_vect[k].avg[i]=avg[i]/float(count);
//        }
        }

    }


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


void bbTracker_t::reproj(float thresh){

    for (unsigned k =0; k<bbox_State_vect.size(); k++){ // for each state
        Utility::bbox<float> predicted_bbox ;
        cv::Point2f bbox_corner[4];
        float err_def,err_temp;
        if (bbox_State_vect[k].lock_proba>1){
         err_temp = calc_err(bbox_State_vect[k].locked_corner);}
        else{
            return;
        }
        if (bbox_State_vect[k].locked_def){
        err_def = calc_err(bbox_State_vect[k].locked_corner_def);
        }else{
             err_def= err_temp+1;
        }
        project_world_to_pixel(bbox_State_vect[k].locked_corner_def, predicted_bbox);
        bbox_to_points_cv(predicted_bbox,bbox_corner );
        cout <<"project on def" << bbox_corner << endl;
        if(err_temp<=err_def){
            project_world_to_pixel(bbox_State_vect[k].locked_corner, predicted_bbox);
            cout<< "############################## We merge with reprojection with last locked! with thresh"<< thresh << endl;

        }else{
            project_world_to_pixel(bbox_State_vect[k].locked_corner_def, predicted_bbox);
            cout<< "############################## We merge with reprojection! with def locked thresh"<< thresh << endl;

        }

        bool iou_rslt= IOU(predicted_bbox,bbox_State_vect[k].cur_detection, thresh );

        if (iou_rslt){
            float xmin, ymin, xmax, ymax;
            xmin = (bbox_State_vect[k].cur_detection.xmin + predicted_bbox.xmin)/2 ;
            xmax = (bbox_State_vect[k].cur_detection.xmax + predicted_bbox.xmax)/2 ;
            ymax = (bbox_State_vect[k].cur_detection.ymax + predicted_bbox.ymax)/2 ;
            ymin = (bbox_State_vect[k].cur_detection.ymin + predicted_bbox.ymin)/2 ;

            if(xmin< xmax && ymin<ymax){

            bbox_State_vect[k].cur_detection.xmin = xmin ;
            bbox_State_vect[k].cur_detection.ymin = ymin ;
            bbox_State_vect[k].cur_detection.xmax = xmax ;
            bbox_State_vect[k].cur_detection.ymax = ymax ;
            bbox_State_vect[k].type_detection = "reproj";
            } else {
                ROS_INFO("REPROJ failed");
            }
        }

    }
}


float bbTracker_t::shift_bbox(Utility::bboxState<float>& bbox_state, cv::Mat new_frame ){
    std::vector<cv::Point2f> corners, corners_forw; // will contain all the bbox tl and br of the images to be shifted
    vector<uchar> status;
    vector<float> err;
    bool matcher=true;

    new_frame.convertTo(new_frame, CV_8UC1);
    prev_frame.convertTo(prev_frame, CV_8UC1);
    cvtColor(new_frame, new_frame, CV_BGR2GRAY);
cout<<"before "<< endl;
   // string ty =  type2str( prev_frame.type() );
   // printf("Matrix: %s %dx%d \n", ty.c_str(), prev_frame.cols, prev_frame.rows );
   // string ty2 =  type2str( new_frame.type() );
   // printf("Matrix: %s %dx%d \n", ty2.c_str(), new_frame.cols, new_frame.rows );
    cv::imwrite( "/home/ubuntu/catkin_vins/src/VINS-Mono/new_frame.jpg", prev_frame );
    cv::imwrite( "/home/ubuntu/catkin_vins/src/VINS-Mono/prev.jpg", new_frame );
    vector<Point2f> old_features, new_feature;
    //goodFeaturesToTrack(prev_frame, old_features, 400, 0.6, 2, Mat(), 2);

    if(bbox_state.age==0 && bbox_state.nb_detected>2){
//        old_features.push_back(cv::Point2f(bbox_state.cur_detection.xmin, bbox_state.cur_detection.ymin)); //tl
//        old_features.push_back(cv::Point2f(bbox_state.cur_detection.xmax, bbox_state.cur_detection.ymin)); //tr
//        old_features.push_back(cv::Point2f(bbox_state.cur_detection.xmin, bbox_state.cur_detection.ymax)); //bl
//        old_features.push_back(cv::Point2f(bbox_state.cur_detection.xmax, bbox_state.cur_detection.ymax)); //br
        corners.push_back(cv::Point2f(bbox_state.cur_detection.xmin, bbox_state.cur_detection.ymin)); //tl
        corners.push_back(cv::Point2f(bbox_state.cur_detection.xmax, bbox_state.cur_detection.ymin)); //tr
        corners.push_back(cv::Point2f(bbox_state.cur_detection.xmin, bbox_state.cur_detection.ymax)); //bl
        corners.push_back(cv::Point2f(bbox_state.cur_detection.xmax, bbox_state.cur_detection.ymax)); //br
        bbox_state.prev_detection =  bbox_state.cur_detection;

    }else if ( bbox_state.nb_detected>2){
//    old_features.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymin));
//    old_features.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymin));
//    old_features.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymax));
//    old_features.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymax));
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymin));
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymin));
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmin, bbox_state.prev_detection.ymax));
    corners.push_back(cv::Point2f(bbox_state.prev_detection.xmax, bbox_state.prev_detection.ymax));

} else {
        cout << "too young for OpticalFlow " << endl;
        return -1;
    }


cv::calcOpticalFlowPyrLK(prev_frame, new_frame, corners, corners_forw, status, err, cv::Size(21, 21), 3);

cv::Mat img_flow=new_frame;
//cout << "size of old feature" << old_features.size()  << " and new feature" << new_feature.size() << endl;
//for (unsigned int i =0; i<4; i++){
// cv::line( img_flow, corners[i], new_feature[i],cv::Scalar(0,255,255));
// if (new_feature.size()>=4){
//    corners_forw.push_back(new_feature[new_feature.size()-(4-i)]);
//}else{
//     return -1;
// }
//}

cv::imwrite( "/home/ubuntu/catkin_vins/src/VINS-Mono/img_flow.jpg", img_flow );
if (status[0] >0 && status[1] >0){
    bbox_state.prev_detection = bbox_state.cur_detection;
bbox_state.cur_detection.xmin = corners_forw[0].x;
bbox_state.cur_detection.ymin = corners_forw[0].y;
bbox_state.cur_detection.xmax = corners_forw[3].x;
bbox_state.cur_detection.ymax = corners_forw[3].y;
cout << "Optical flow to update the bbox " << bbox_state.bbox_id << endl;
bbox_state.type_detection= "optical";
cout<<"after "<< endl;

return 0;
} else if (matcher){
    cout << "Error in Optical FLow for bbox state id" << bbox_state.bbox_id << endl;
    //Try ORB instead
    //cvtColor(roi_prev, roi_prev, CV_BGR2GRAY);


    // Variables to store keypoints and descriptors
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    // Detect ORB features and compute descriptors.
  Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);
     detector->detectAndCompute(prev_frame, Mat(), keypoints1, descriptors1);
     detector->detectAndCompute(new_frame, Mat(), keypoints2, descriptors2);
    // Match features.
     FlannBasedMatcher matcher;
     std::vector< DMatch > matches;
  matcher.match( descriptors1, descriptors2, matches );
    // Sort matches by score
    std::sort(matches.begin(), matches.end());

    // Remove not so good matches
    const int numGoodMatches = matches.size() * 0.15f;
    matches.erase(matches.begin()+numGoodMatches, matches.end());

    // Extract location of good matches
    std::vector<Point2f> points1, points2;

    for( size_t i = 0; i < matches.size(); i++ )
    {
      points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
      points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
    }
    cv::Mat img_matches;
    cv::drawMatches( prev_frame, keypoints1, new_frame, keypoints2,
                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imwrite( "/home/ubuntu/catkin_vins/src/VINS-Mono/matches.jpg", img_matches );
    if (points1.size()>3 && points2.size()>3 ){
    // Find homography
    cv::Mat h = findHomography(points1, points2, RANSAC);

    //cout << "ok homo" << h << "corners" << corners << " " << corners_forw << "size" << new_frame.size() << endl;
    // Use homography to warp image
    if (!h.empty()){
    perspectiveTransform(corners, corners_forw, h);

    bbox_state.prev_detection = bbox_state.cur_detection;
    bbox_state.cur_detection.xmin = corners_forw[0].x;
    bbox_state.cur_detection.ymin = corners_forw[0].y;
    bbox_state.cur_detection.xmax = corners_forw[3].x;
    bbox_state.cur_detection.ymax = corners_forw[3].y;
    cout << "Optical flow to update the bbox with surf " << bbox_state.bbox_id << endl;
    bbox_state.type_detection= "matching";


    return 1;}

    }
}
else{
        cout << "SURF or opticalFlow also failed" << endl;
        return -1;
    }


}





void bbTracker_t::shift_all(cv::Mat new_frame, cv::Mat &output_frame){ //shift the bbox with coming frame

    for (unsigned k=0; k<bbox_State_vect.size(); k++) { //for each box
        cv::Point2f bbox_corner[4];
        if(prev_frame.empty()){
            prev_frame=cur_frame;
            return;
        }


        if (bbox_State_vect[k].age==0){
        bbox_to_points_cv(bbox_State_vect[k].cur_detection, bbox_corner );
}else{
        bbox_to_points_cv(bbox_State_vect[k].prev_detection, bbox_corner );
        }
     //bbTracker_.bbox_to_points_cv(bbTracker_.bbox_State_vect[i].cur_detection, bbox_corner );
       // cout << "debug in shift bbox_corner[0]" << bbox_corner[0] << "  bbox_corner[0]" << bbox_corner[0] << endl;
        //if ( bbox_corner[0].x <= bbox_corner[3].x && bbox_corner[0].y <= bbox_corner[3].y){

     cv::rectangle(output_frame, bbox_corner[0], bbox_corner[3],cv::Scalar(0, 0, 255), 2, 8, 0) ;
//} else {
//            return;
//        }
float result = shift_bbox(bbox_State_vect[k],new_frame );
//cout << "result of shift bbox" << result << endl;

if (result>=0  && result <2 && bbox_corner[0].x <= bbox_corner[3].x && bbox_corner[0].y <= bbox_corner[3].y){
    prev_frame=cur_frame;
    bbox_to_points_cv(bbox_State_vect[k].cur_detection, bbox_corner );
    cv::rectangle(output_frame, bbox_corner[0], bbox_corner[3],cv::Scalar(0, 255, 255), 2, 8, 0) ;

putText(output_frame, std::to_string(bbox_State_vect[k].bbox_id), Point2f(100+10*k,100+10*k), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,255,255), 2 );

if (result==1)
        putText(output_frame, "SURF", Point2f(300+10*k,200+10*k), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,255), 2 );
if (result==0)
        putText(output_frame, "Optical", Point2f(300+10*k,200+10*k), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,255), 2 );
if (result==-1)
        putText(output_frame, "fail", Point2f(300+10*k,200+10*k), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,255), 2 );

}
}
    //multiTracker->update(new_frame);

      // Draw tracked objects
    //  for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
    //  {
        //rectangle(output_frame, multiTracker->getObjects()[i], colors[i], 2, 1);
        //cout <<"in multiTRacking for i " << i << "we have " << multiTracker->getObjects()[i] << endl;
//      }


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
//                        std::cout <<std::setprecision(10) <<" in update_id change from id" <<  bbox_State_vect[k].feature_id[i]  << "to" <<  id[i]
//                                 << " and bbox numero  "<< box_nb<<" in the bboxState_vec,and id " <<  bbox_State_vect[k].bbox_id << "for corner: " << i
//                                <<  std::endl;
                    bbox_State_vect[k].feature_id[i] = id[i]; // otherwise stay 0


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
            if(bbox_State_vect[i].age >max_age_detection+ int(bbox_State_vect[i].lock_proba/max_age_detection) ){
                cout << "-------we erase bbox nb" << i << " and id" << bbox_State_vect[i].bbox_id <<"from bbox state" << endl;

                bbox_State_vect.erase(bbox_State_vect.begin()+i);
             if (bbox_State_vect[i].age >max_age_detection){
                cout <<"============= The locked bbox saved the detection beacause age is" <<bbox_State_vect[i].age << "and thresh is " << int(bbox_State_vect[i].lock_proba/max_age_detection) <<  endl;
            }}
        }
            //std::cout << "the predicted bbox is " << frame_count <<"frame year old and contains"<< bbox_State_vect.size() << "boxes" << endl;


}


void bbTracker_t::nonMaxSupp(){
    for (unsigned k=0; k<bbox_State_vect.size(); k++) {
        for (unsigned i=k+1; i<bbox_State_vect.size(); i++) {
                if(IOU(bbox_State_vect[k].cur_detection, bbox_State_vect[i].cur_detection, 0.4)){
                    if (bbox_State_vect[k].nb_detected > bbox_State_vect[i].nb_detected){
                        bbox_State_vect.erase(bbox_State_vect.begin()+i);

                    }

                }
                if (i==k){
                    return;
                }
        }
    }

                             }

bbTracker_t::bbTracker_t()
{

 init_=false;
 z_C=1;
 bbox_State_vect.clear();
 thresh = 0.4;
 thresh_pixel = 30;
 max_age_detection=3; //increase every time a detection occur without have been updated  min_hit=0;
 max_age_frame =5;
 frame_count=0;
 id_count=0;
  c_u=363.0;
  c_v=248.1 ;

  f_u=461.6;
  f_v=460.3;
  trackerType ="CSRT";
  //multiTracker = cv::MultiTracker::create();
  cv::RNG rng(0);
    for(int i=0; i < 10; i++)
      colors.push_back(Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));

}

//cv::Ptr<Tracker> bbTracker_t::createTrackerByName()
//{
//  cv::Ptr<Tracker> tracker;
//  if (trackerType ==  trackerTypes[0])
//    tracker = TrackerBoosting::create();
//  else if (trackerType == trackerTypes[1])
//    tracker = TrackerMIL::create();
//  else if (trackerType == trackerTypes[2])
//    tracker = TrackerKCF::create();
//  else if (trackerType == trackerTypes[3])
//    tracker = TrackerTLD::create();
//  else if (trackerType == trackerTypes[4])
//    tracker = TrackerMedianFlow::create();
//  else if (trackerType == trackerTypes[5])
//    tracker = TrackerGOTURN::create();
//  else if (trackerType == trackerTypes[6])
//    tracker = TrackerMOSSE::create();
//  else if (trackerType == trackerTypes[7])
//    tracker = TrackerCSRT::create();
//  else {
//    cout << "Incorrect tracker name" << endl;
//    cout << "Available trackers are: " << endl;
//    for (vector<string>::iterator it = trackerTypes.begin() ; it != trackerTypes.end(); ++it)
//      std::cout << " " << *it << endl;
//  }
//  return tracker;
//}





