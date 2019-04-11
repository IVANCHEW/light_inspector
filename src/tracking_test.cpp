#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <iostream>
#include <cstring>

const int slider_max = 100;
const int threshold_max = 255;
const int threshold_type = 3;
int slider_hue, slider_threshold;
int hue_filter, threshold_filter;

bool initialisation = true;
std::string trackerType = "KCF";
cv::Ptr<cv::MultiTracker> multiTracker = cv::MultiTracker::create();
std::vector<std::string> trackerTypes;

void on_trackbar_hue( int, void* )
{
 hue_filter = (double) slider_hue/slider_max ;
}

void on_trackbar_threshold( int, void* )
{
 threshold_filter = slider_threshold;
}

// create tracker by name
cv::Ptr<cv::Tracker> createTrackerByName(std::string trackerType) 
{
  cv::Ptr<cv::Tracker> tracker;
  if (trackerType ==  trackerTypes[0])
    tracker = cv::TrackerBoosting::create();
  else if (trackerType == trackerTypes[1])
    tracker = cv::TrackerMIL::create();
  else if (trackerType == trackerTypes[2])
    tracker = cv::TrackerKCF::create();
  else if (trackerType == trackerTypes[3])
    tracker = cv::TrackerTLD::create();
  else if (trackerType == trackerTypes[4])
    tracker = cv::TrackerMedianFlow::create();
  else if (trackerType == trackerTypes[5])
    tracker = cv::TrackerGOTURN::create();
  else if (trackerType == trackerTypes[6])
    tracker = cv::TrackerMOSSE::create();
  //~ else if (trackerType == trackerTypes[7])
    //~ tracker = cv::TrackerCSRT::create();
  else {
    ROS_DEBUG_STREAM("Incorrect tracker name");
    ROS_DEBUG_STREAM("Available trackers are: ");
    for (std::vector<std::string>::iterator it = trackerTypes.begin() ; it != trackerTypes.end(); ++it)
      ROS_DEBUG_STREAM(" " << *it);
  }
  return tracker;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat image;
  cv::Mat image_grey;
  cv::Mat image_threshold;
  cv::Mat image_canny;
  cv::Mat image_drawing;
  cv::Mat image_tracker;
  
  int thresh = 100;
  int max_thresh = 255;
  cv::RNG rng(12345);
  
  cv::Scalar color_red = cv::Scalar(0,0,255);
  cv::Scalar color_blue = cv::Scalar(0,255,0); //Color is BGR
  
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    image_drawing = image.clone();
    cv::cvtColor( image, image_grey, CV_BGR2GRAY);
    cv::threshold( image_grey, image_threshold, threshold_filter, threshold_max,threshold_type );
    cv::blur( image_threshold, image_threshold, cv::Size(3,3) );
    cv::Canny( image_threshold, image_canny, thresh, thresh*2, 3 );
    cv::findContours( image_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );    
    image_tracker = cv::Mat::zeros( image_canny.size(), CV_8UC3 );
    
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect2d> boundRect( contours.size() );
    //~ std::vector<cv::Point2f> center( contours.size() );
    //~ std::vector<float> radius( contours.size() );
    
    for( int i = 0; i< contours.size(); i++ ){
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true);
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    }
    
    for( int i = 0; i< contours.size(); i++ )
    {
      //~ cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ); //For random colors      
      //~ cv::drawContours( image_drawing, contours, i, color_blue, 2, 8, hierarchy, 0, cv::Point() );      
      //~ cv::rectangle( image_drawing, boundRect[i].tl(), boundRect[i].br(), color_red, 2, 8, 0);  
      cv::rectangle( image_drawing, boundRect[i].tl(), boundRect[i].br(), color_red, 1, 8, 0);    
    }
    
    if (initialisation){
      initialisation = false;
      ROS_DEBUG_STREAM("Initialisation Contour Size: " << contours.size());
      for( int i = 0; i< contours.size(); i++ ){
        multiTracker->add(createTrackerByName(trackerType), image_grey, boundRect[i]);
      }
    } else {
      multiTracker->update(image_grey);
      ROS_DEBUG_STREAM("Tracking Object Size: " << multiTracker->getObjects().size());
      for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
      {
        cv::Point rect_center = ((multiTracker->getObjects()[i].br() + multiTracker->getObjects()[i].tl())*0.5);
        cv::circle(image_tracker, rect_center, 5, color_red, 5, 8, 0); 
        ROS_DEBUG_STREAM("Object " << i << " Point: " << rect_center);
      }
    }
    
    //Test
    cv::Point image_center;
    image_center.x = image_tracker.cols/2;
    image_center.y = image_tracker.rows/2;
    //~ cv::circle(image_tracker, image_center, 5, color_red, 5, 8, 0); 
    //~ ROS_DEBUG_STREAM("Image Center " << image_center);
    
    //DEBUG
    ROS_DEBUG_STREAM("Threshold value: " << threshold_filter);
    
    cv::imshow("view", image_drawing);
    cv::imshow("original", image);
    cv::imshow("tracker", image_threshold);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  int windows_reduction = 2;
  
  cv::namedWindow("view", cv::WINDOW_NORMAL);
  cv::moveWindow("view", 0,0);
  cv::resizeWindow("view", 1712/windows_reduction,960/windows_reduction);
  
  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::moveWindow("original", 2000/windows_reduction,0);
  cv::resizeWindow("original", 1712/windows_reduction,960/windows_reduction);

  cv::namedWindow("tracker", cv::WINDOW_NORMAL);
  cv::moveWindow("tracker", 0,1100/windows_reduction);
  cv::resizeWindow("tracker", 1712/windows_reduction,960/windows_reduction);
  
  cv::startWindowThread();
  
  slider_hue=0;
  slider_threshold=50;
  cv::createTrackbar("Hue Filter", "view", &slider_hue, slider_max, on_trackbar_hue );
  cv::createTrackbar("Threshold Filter", "view", &slider_threshold, threshold_max, on_trackbar_threshold );
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/videofile/image_raw", 1, imageCallback);
  
  trackerTypes.push_back("BOOSTING");
  trackerTypes.push_back("MIL");
  trackerTypes.push_back("KCF");
  trackerTypes.push_back("TLD");
  trackerTypes.push_back("MEDIANFLOW");
  trackerTypes.push_back("GOTURN");
  trackerTypes.push_back("MOSSE");
  trackerTypes.push_back("CSRT");

  ros::spin();
  cv::destroyWindow("view");
}
  
