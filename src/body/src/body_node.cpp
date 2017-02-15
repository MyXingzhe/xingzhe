#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "body.h"

const double UPDATE_RATE = 50; // desired publication rate of IMU data

int main(int argc, char **argv)
{
	// Set up ROS node.
	ros::init(argc, argv, "Body");
	ros::NodeHandle n;
	int rate = 2;
    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

	// message used for publishing actuator control value
	std_msgs::Float32 actuator_msg;

	ros::NodeHandle private_node_handle_("~");
	ros::Publisher actuator_pub = n.advertise<std_msgs::Float32>("Distance", 1);

	// Tell ROS how fast to run this node.
	ros::Rate r(rate);

	ROS_INFO("Start Body Node");

    ros::Rate sleep_timer(UPDATE_RATE/2);  // a timer for desired rate, 50Hz is a good speed. We set to half for 2 seperate sleeps

	// Main loop.
	while (n.ok())
	{
	}

	return 0;
}
