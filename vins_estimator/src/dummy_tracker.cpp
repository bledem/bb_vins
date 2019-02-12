#include <opencv2/core/ocl.hpp>
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

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


cv::Mat frame;
string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
// vector <string> trackerTypes(types, std::end(types));

// Create a tracker
string trackerType = trackerTypes[2];

Ptr<Tracker> tracker= TrackerTLD::create();
//if (CV_MINOR_VERSION < 3)
//{
//    Ptr<Tracker> tracker = Tracker::create(trackerType);
//}
//else
//{
//    if (trackerType == "BOOSTING")
//        tracker = TrackerBoosting::create();
//    if (trackerType == "MIL")
//        tracker = TrackerMIL::create();
//    if (trackerType == "KCF")
//        tracker = TrackerKCF::create();
//    if (trackerType == "TLD")
//        tracker = TrackerTLD::create();
//    if (trackerType == "MEDIANFLOW")
//        tracker = TrackerMedianFlow::create();
//    if (trackerType == "GOTURN")
//        tracker = TrackerGOTURN::create();
//    if (trackerType == "MOSSE")
//        tracker = TrackerMOSSE::create();
////        if (trackerType == "CSRT")
////            tracker = TrackerCSRT::create();
//}
// Read video
//VideoCapture video("videos/chaplin.mp4");


ros::Publisher  optical_image_pub_ ;

// Define initial bounding box
Rect2d bbox(287, 23, 86, 320);
bool first=true;

void img2_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr optical_result_bridge;


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
        optical_result_bridge =  cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }else{
        optical_result_bridge =  cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    }
    //ROS_INFO("RECEIVING IMG");

        //bbTracker_.shift_frame(out_img->image); //shift the detected points from prev_frame
    frame = optical_result_bridge->image;

    if(first){
        rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );

        if(frame.cols>0){
        first-false;
        tracker->init(frame, bbox);
    }
    }else {
        // Start timer
        double timer = (double)getTickCount();

        // Update the tracking result
        bool ok = tracker->update(frame, bbox);

        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);

        if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    optical_image_pub_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::Subscriber sub_cam_image = n.subscribe("/hummingbird/vi_sensor/left/image_raw", 2000, img2_callback);
    // List of tracker types in OpenCV 3.4.1
    optical_image_pub_ = n.advertise<sensor_msgs::Image>("dummy_tracking",2000); //for the bbox check


    // Uncomment the line below to select a different bounding box
    // bbox = selectROI(frame, false);
    // Display bounding box.



        ros::spin();

        return 0;
}
