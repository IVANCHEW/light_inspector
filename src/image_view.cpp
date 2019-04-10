#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image; 
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_view");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  int windows_reduction = 2;
  cv::namedWindow("view", cv::WINDOW_NORMAL);
  cv::moveWindow("view", 0,0);
  cv::resizeWindow("view", 1712/windows_reduction,960/windows_reduction);
  
  cv::startWindowThread();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/bebop/image_raw", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}
  
