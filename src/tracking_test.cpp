#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat image;
  cv::Mat image_grey;
  cv::Mat image_canny;
  cv::Mat image_drawing;
  
  int thresh = 100;
  int max_thresh = 255;
  cv::RNG rng(12345);

  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor( image, image_grey, CV_BGR2GRAY);
    cv::blur( image_grey, image_grey, cv::Size(3,3) );
    cv::Canny( image_grey, image_canny, thresh, thresh*2, 3 );
    cv::findContours( image_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    
    image_drawing = cv::Mat::zeros( image_canny.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
      //~ cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ); //For random colors
      cv::Scalar color = cv::Scalar(0,255,0); //Color is BGR
      cv::drawContours( image_drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    }

    cv::imshow("view", image_drawing);
    cv::imshow("original", image);
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
  cv::namedWindow("view", cv::WINDOW_NORMAL);
  cv::moveWindow("view", 0,0);
  cv::resizeWindow("view", 1712,960);
  
  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::moveWindow("original", 2000,0);
  cv::resizeWindow("original", 1712,960);
  
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/videofile/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
  
