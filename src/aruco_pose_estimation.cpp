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
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

//Parameters for pose estimation
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Mat camera_matrix;
cv::Mat dist_coeffs;
cv::Mat tvec;
cv::Mat rvec;
int landing_marker_id;
std_msgs::Bool landing_marker;
geometry_msgs::PoseStamped pose_msg;
  
//Parameters for general ROS functions
float aruco_size; 
int windows_reduction;
std::string yaml_path_;
std::string package_path_;
std::string point_cloud_topic_name;
std::string image_topic_name;
std::string camera_name;
ros::Publisher image_pub_;
ros::Publisher landing_bool_pub_;
ros::Publisher landing_tvec_pub_;
int counter;

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
	//~ ROS_DEBUG_STREAM("Number of markers detected: " << marker_ids.size());
	landing_marker.data = false;
	if (marker_ids.size() > 0){
		for (int i = 0 ; i < marker_ids.size() ; i++){
			//~ ROS_DEBUG_STREAM("Marker ID found: " << marker_ids[i] );
			if (marker_ids[i]==landing_marker_id){
				std::vector< std::vector<cv::Point2f> > single_corner(1);
				single_corner[0] = marker_corners[i];			
				cv::aruco::estimatePoseSingleMarkers(single_corner, aruco_square_size, mtx, dist, rvec, tvec);
				cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
				cv::aruco::drawAxis(input_image, mtx, dist, rvec, tvec, aruco_square_size/2);
				
				//Annotate image with text
				std::stringstream sstm;
				std::string sstm_result;
				sstm << "X: " << tvec.at<double>(0);
				sstm_result = sstm.str();
				cv::putText(input_image, sstm_result, cv::Point2f(20,20), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,255,0,255));
				sstm.str("");
				sstm << "Y:" << tvec.at<double>(1);
				sstm_result = sstm.str();
				cv::putText(input_image, sstm_result, cv::Point2f(20,40), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,255,0,255));
				sstm.str("");
				sstm << "Z: " << tvec.at<double>(2);				
				sstm_result = sstm.str();
				cv::putText(input_image, sstm_result, cv::Point2f(20,60), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,255,0,255));
				
				//Prepare ROS messages
				landing_marker.data = true;
				pose_msg.pose.position.x = tvec.at<double>(0);
				pose_msg.pose.position.y = tvec.at<double>(1);
				pose_msg.pose.position.z = tvec.at<double>(2);
				ROS_DEBUG_STREAM("Landing Marker Found");	
				ROS_DEBUG_STREAM(tvec);			
			}
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
    arucoPoseEstimation(image, 0, tvec, rvec, camera_matrix, dist_coeffs, true);
	
	//Publish annotated Image
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg; // >> message to be sent
	std_msgs::Header header; // empty header
	header.seq = counter; // user defined counter
	header.stamp = ros::Time::now(); // time
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    image_pub_.publish(img_msg);
    
    
    //Publish landing data
    pose_msg.header.frame_id = "aruco_marker";
    landing_bool_pub_.publish(landing_marker);
    landing_tvec_pub_.publish(pose_msg);
    cv::imshow("view", image);
    cv::waitKey(30);
    counter++;
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
	nh_private_.getParam("landing_marker_id", landing_marker_id);
	
	//Initialise pose estimation parameters	
	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
	camera_matrix.at<double>(2,2) = 1;	
	loadCalibrationMatrix(camera_name);
	counter = 0;
	
	//Initialise OpenCV displays
	cv::namedWindow("view", cv::WINDOW_NORMAL);
	cv::moveWindow("view", 0,0);
	cv::resizeWindow("view", 1712/windows_reduction,960/windows_reduction);

	cv::startWindowThread();

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(image_topic_name, 1, imageCallback);
	image_pub_ = nh.advertise<sensor_msgs::Image>("/annotated_image/image", 1);
	landing_bool_pub_ = nh.advertise<std_msgs::Bool>("/landing/marker_found",1);
	landing_tvec_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/landing/marker_pose",1);
	ros::spin();
	cv::destroyWindow("view");
}
  
