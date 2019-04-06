#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

ros::Publisher landing_pub;
ros::Publisher takeoff_pub;
ros::Publisher cmd_vel_pub;
std::string yaml_path_;
std::string package_path_;
std::string image_topic_ 	= "/bebop/image_raw";
std::string odom_topic_ 	= "/bebop/odom";
std::string takeoff_topic_ 	= "/bebop/takeoff";
std::string landing_topic_ 	= "/bebop/land";
std::string cmd_vel_topic_ 	= "/bebop/cmd_vel";
bool debug_ = true;
geometry_msgs::Twist geo_msg;
std_msgs::Empty empty_msg;

void take_off_command(){
  ROS_DEBUG_STREAM("Initiate Takeoff");
  takeoff_pub.publish(empty_msg);
}

void land_command(){
  ROS_DEBUG_STREAM("Initiate Landing");
  landing_pub.publish(empty_msg);
}

void hover(){
  ROS_DEBUG_STREAM("Hover");
  geo_msg.linear.x = 0;
  geo_msg.linear.y = 0;
  geo_msg.linear.z = 0;
  cmd_vel_pub.publish(geo_msg);
}

void update_vel_command(){
  ROS_DEBUG_STREAM("Update Velocity");
  cmd_vel_pub.publish(geo_msg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_DEBUG_STREAM("Command Velocity Callback");
  geo_msg.linear.x = msg->linear.x;
	geo_msg.linear.y = msg->linear.y;
	geo_msg.linear.z = msg->linear.z;
  geo_msg.angular.z = msg->angular.z;
}

void takeOffCallback(const std_msgs::Empty::ConstPtr& received_msg)
{
  ROS_DEBUG_STREAM("Take Off Callback");
  take_off_command();
}

void landingCallback(const std_msgs::Empty::ConstPtr& received_msg)
{
  ROS_DEBUG_STREAM("Landing Callback");
  land_command();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_bebop_interface");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ROS_INFO("Node initialised");
  
  ros::NodeHandle nh_;

	landing_pub = nh_.advertise<std_msgs::Empty>(landing_topic_, 1);
	takeoff_pub = nh_.advertise<std_msgs::Empty>(takeoff_topic_, 1);
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  
  ros::Subscriber landing_pub = nh_.subscribe<std_msgs::Empty>("/bebop_joy/land", 10, landingCallback);
  ros::Subscriber takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("/bebop_joy/takeoff", 10, takeOffCallback);
  ros::Subscriber cmd_update_sub_ = nh_.subscribe<geometry_msgs::Twist>("/bebop_joy/cmd_vel", 10, cmdVelCallback);
  
  ros::Rate loop_rate(20);
  
  while (ros::ok()){
    
    ros::spinOnce();
    update_vel_command();
    loop_rate.sleep();
  }
  
}
