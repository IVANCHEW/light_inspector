#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void LandingBoolCallback(const std_msgs::Bool bool_msg_);
	void LandingTvecCallback(const geometry_msgs::PoseStamped pose_msg);
	
	ros::NodeHandle nh_;
	std::string takeoff_topic_ 	= "/bebop_joy/takeoff";
	std::string landing_topic_ 	= "/bebop_joy/land";
	std::string cmd_vel_topic_ 	= "/bebop_joy/cmd_vel";
	std::string auto_pilot_topic_ = "/bebop_joy/autopilot";
	std::string tracking_topic_ = "/bebop_joy/object_tracking";
	bool debug_ = true;
	bool active_object_track_ = false;
	bool active_track_ = false;
	bool track_marker_available_ = false;
	int linear_, angular_;
	double l_scale_, a_scale_;
	float cmd_vel_increment_;
	ros::Publisher vel_pub_;
	ros::Publisher takeoff_pub;
	ros::Publisher landing_pub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher auto_pilot_pub;
	ros::Publisher initiate_object_track_pub;
	geometry_msgs::Twist geo_msg;
	std_msgs::Empty empty_msg;
	std_msgs::String auto_pilot_file_;
	std_msgs::Bool object_tracking_msg;
	ros::Subscriber joy_sub_;
	ros::Subscriber landing_bool_sub_;
	ros::Subscriber landing_tvec_sub_;
	
	void take_off_command(){
		ROS_INFO("Initiate Takeoff");
		takeoff_pub.publish(empty_msg);
	}

	void land_command(){
		ROS_INFO("Initiate Landing");
		landing_pub.publish(empty_msg);
	}
	
	void auto_pilot_command(){
		ROS_INFO("Initiate Autopilot");
		auto_pilot_file_.data = "autopilot.mavlink";
		auto_pilot_pub.publish(auto_pilot_file_);
	}
	
	void hover(){
		ROS_INFO("Hover");
		geo_msg.linear.x = 0;
		geo_msg.linear.y = 0;
		geo_msg.linear.z = 0;
		cmd_vel_pub.publish(geo_msg);
	}
	
	void update_vel_command(){
		cmd_vel_pub.publish(geo_msg);
	}
	
	void track_landing_command(){		
		if(track_marker_available_){
			ROS_INFO("Initiate Track Landing");
			active_track_ = true;	
		}	
	}
	
	void cancel_track_landing(){
		ROS_INFO("Cancel Track Landing");
		active_track_ = false;
	}
	
	void calculate_track_action(){
		
	}
	
	void object_tracking_toggle(){
		active_object_track_ = !active_object_track_;
		ROS_DEBUG_STREAM("Object Tracking toggled: " << active_object_track_);
		object_tracking_msg.data = active_object_track_;
		initiate_object_track_pub.publish(object_tracking_msg);
	}

};


TeleopTurtle::TeleopTurtle()
{
	landing_pub = nh_.advertise<std_msgs::Empty>(landing_topic_, 1);
	takeoff_pub = nh_.advertise<std_msgs::Empty>(takeoff_topic_, 1);
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
	auto_pilot_pub = nh_.advertise<std_msgs::String>(auto_pilot_topic_, 1);	
	initiate_object_track_pub = nh_.advertise<std_msgs::Bool>(tracking_topic_, 1);	
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
	landing_bool_sub_ = nh_.subscribe<std_msgs::Bool>("/landing/marker_found", 1, &TeleopTurtle::LandingBoolCallback, this);
	landing_tvec_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/landing/marker_pose", 1, &TeleopTurtle::LandingTvecCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	
	if (!active_track_){
		geo_msg.linear.x = joy->axes[1];
		geo_msg.linear.y = joy->axes[0];
		geo_msg.linear.z = joy->axes[5];
		geo_msg.angular.z = joy->axes[2];
	} else {
		calculate_track_action();
	}
	
	ROS_DEBUG_STREAM(geo_msg);
	
	if (joy->buttons[2] == 1){ //Circle
		take_off_command();		
	} else if (joy->buttons[0] == 1){ //Square
		track_landing_command();
	} else if (joy->buttons[3] == 1){ //Triangle
		cancel_track_landing();
	} else if (joy->buttons[1] == 1){ //Cross
		land_command();
	} else if (joy->buttons[4] == 1 && joy->buttons[5] == 1){ //R1 and L1
		auto_pilot_command();
	} else if (joy->buttons[4] == 1){
		object_tracking_toggle();
	}
	update_vel_command();	
}

void TeleopTurtle::LandingBoolCallback(const std_msgs::Bool bool_msg_)
{
	track_marker_available_ = bool_msg_.data;
}

void TeleopTurtle::LandingTvecCallback(const geometry_msgs::PoseStamped pose_msg)
{
	if (track_marker_available_ && active_track_){
		
		//Forward movement
		if (pose_msg.pose.position.z > 0.8){
			geo_msg.linear.x = 0.2;
		} else {
			geo_msg.linear.x = 0;
		}
		
		//Vertical movement
		if (pose_msg.pose.position.y > 0.2){
			geo_msg.linear.z = -0.2;
		} else if (pose_msg.pose.position.y < -0.2) {
			geo_msg.linear.z = 0.2;
		} else {
			geo_msg.linear.z = 0;
		}
		
		//Sideways movement
		if (pose_msg.pose.position.x > 0.2){
			geo_msg.linear.y = -0.2;
		} else if (pose_msg.pose.position.x < -0.2) {
			geo_msg.linear.y = 0.2;
		} else {
			geo_msg.linear.y = 0;
		}
	}	
	update_vel_command();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ROS_INFO("Node initialised");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
