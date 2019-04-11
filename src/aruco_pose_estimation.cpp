#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>

//Parameters for pose estimation
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Mat camera_matrix;
cv::Mat dist_coeffs;
cv::Mat tvec;
cv::Mat rvec;

//Parameters for general ROS functions
float aruco_size; 
int windows_reduction;
std::string yaml_path_;
std::string package_path_;
std::string point_cloud_topic_name;
std::string image_topic_name;
std::string camera_name;

//Parameter for video recording
cv::VideoWriter video;

void updateParameters(YAML::Node config){
	std::cout << "Updating Camera Parameters" << std::endl;
	if (config["fx"])
		camera_matrix.at<double>(0,0) = config["fx"].as<double>();
	if (config["fx"])
		camera_matrix.at<double>(1,1) = config["fx"].as<double>();
	if (config["x0"])
		camera_matrix.at<double>(0,2) = config["x0"].as<double>();
	if (config["y0"])
		camera_matrix.at<double>(1,2) = config["y0"].as<double>();
	if (config["k1"])
		dist_coeffs.at<double>(0,0) = config["k1"].as<double>();
	if (config["k2"])
		dist_coeffs.at<double>(1,0) = config["k2"].as<double>();
	if (config["k3"])
		dist_coeffs.at<double>(4,0) = config["k3"].as<double>();
	if (config["p1"])
		dist_coeffs.at<double>(2,0) = config["p1"].as<double>();
	if (config["p2"])
		dist_coeffs.at<double>(3,0) = config["p2"].as<double>();
	if (config["image_topic"])
		image_topic_name = config["image_topic"].as<std::string>();
	if (config["cloud_topic"])
		point_cloud_topic_name = config["cloud_topic"].as<std::string>();
}

void loadCalibrationMatrix(std::string camera_name_){
	// Updates Parameter with .yaml file
	yaml_path_ = package_path_ + "/config/camera_info.yaml";
	YAML::Node config;
	try 
	{
		config = YAML::LoadFile(yaml_path_);
	} 
	catch (YAML::Exception &e) 
	{
		ROS_ERROR_STREAM("YAML Exception: " << e.what());
		exit(EXIT_FAILURE);
	}
	if (!config[camera_name_])
	{
		ROS_ERROR("Cannot find default parameters in yaml file: %s", yaml_path_.c_str());
		exit(EXIT_FAILURE);
	}  
	updateParameters(config[camera_name_]);
}

bool arucoPoseEstimation(cv::Mat& input_image, int id, cv::Mat& tvec, cv::Mat& rvec, cv::Mat& mtx, cv::Mat& dist, bool draw_axis){
	// Contextual Parameters
	float aruco_square_size = aruco_size;
	bool marker_found = false;
	std::vector< int > marker_ids;
	std::vector< std::vector<cv::Point2f> > marker_corners, rejected_candidates;
	cv::Mat gray;
	
	cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
	cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);	
	ROS_DEBUG_STREAM("Number of markers detected: " << marker_ids.size());
	if (marker_ids.size() > 0){
		for (int i = 0 ; i < marker_ids.size() ; i++){
			ROS_DEBUG_STREAM("Marker ID found: " << marker_ids[i] );
			
			std::vector< std::vector<cv::Point2f> > single_corner(1);
			single_corner[0] = marker_corners[i];			
			cv::aruco::estimatePoseSingleMarkers(single_corner, aruco_square_size, mtx, dist, rvec, tvec);
			cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
			cv::aruco::drawAxis(input_image, mtx, dist, rvec, tvec, aruco_square_size/2);
		}
		marker_found = true;
	}
	
	return marker_found;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image; 
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    ROS_DEBUG_STREAM(image.size());
    arucoPoseEstimation(image, 0, tvec, rvec, camera_matrix, dist_coeffs, true);
    video.write(image);
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
	//Initialise ROS parameters
	ros::init(argc, argv, "aruco_pose_estimation");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private_("~");
	package_path_ = ros::package::getPath("light_inspector");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	
	//Get launch file parameters
	nh_private_.getParam("aruco_size", aruco_size);
	nh_private_.getParam("windows_reduction", windows_reduction);
	nh_private_.getParam("camera_name", camera_name);
	
	//Initialise pose estimation parameters	
	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
	camera_matrix.at<double>(2,2) = 1;	
	loadCalibrationMatrix(camera_name);
	
	//Initialise video recording parameters
	//NOTE: Video size currently defined statically
	cv::VideoWriter video(package_path_ + "/output.avi" ,CV_FOURCC('M','J','P','G'),30, cv::Size(856,480));
	
	//Initialise OpenCV displays
	cv::namedWindow("view", cv::WINDOW_NORMAL);
	cv::moveWindow("view", 0,0);
	cv::resizeWindow("view", 1712/windows_reduction,960/windows_reduction);

	cv::startWindowThread();

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(image_topic_name, 1, imageCallback);

	ros::spin();
	cv::destroyWindow("view");
	video.release();
}
  
