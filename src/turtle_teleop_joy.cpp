#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;
	std::string takeoff_topic_ 	= "/bebop_joy/takeoff";
	std::string landing_topic_ 	= "/bebop_joy/land";
	std::string cmd_vel_topic_ 	= "/bebop_joy/cmd_vel";
	bool debug_ = true;
	int linear_, angular_;
	double l_scale_, a_scale_;
	float cmd_vel_increment_;
	ros::Publisher vel_pub_;
	ros::Publisher takeoff_pub;
	ros::Publisher landing_pub;
	ros::Publisher cmd_vel_pub;
	geometry_msgs::Twist geo_msg;
	std_msgs::Empty empty_msg;
	ros::Subscriber joy_sub_;
	
	void take_off_command(){
		ROS_INFO("Initiate Takeoff");
		takeoff_pub.publish(empty_msg);
	}

	void land_command(){
		ROS_INFO("Initiate Landing");
		landing_pub.publish(empty_msg);
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

};


TeleopTurtle::TeleopTurtle()
{
	landing_pub = nh_.advertise<std_msgs::Empty>(landing_topic_, 1);
	takeoff_pub = nh_.advertise<std_msgs::Empty>(takeoff_topic_, 1);
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	geo_msg.linear.x = joy->axes[1];
	geo_msg.linear.y = joy->axes[0];
	//Second joy stick left-right
	geo_msg.angular.z = joy->axes[3];
	//Second joy stick up-down
	geo_msg.linear.z = joy->axes[4];
	
	ROS_DEBUG_STREAM(geo_msg);
	
	if (joy->buttons[1] == 1){
	  //Right Palm Cirlce Button
	  ROS_DEBUG_STREAM("Button 1 pressed");
	  take_off_command();
	} else if (joy->buttons[2] == 1){
	  //Right Palm Triangle Button
	  ROS_DEBUG_STREAM("Button 2 pressed");
	  hover();
	} else if (joy->buttons[3] == 1){
	  //Right Palm Square Button
	  ROS_DEBUG_STREAM("Button 3 pressed");
	} else if (joy->buttons[0] == 1){
	  //Right Palm Cross Button
	  land_command();
	  ROS_DEBUG_STREAM("Button 0 pressed");
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
