#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "light.h"

int main(int argc, char **argv)
{
	// Set up ROS node.
	ros::init(argc, argv, "Light");
	ros::NodeHandle n;
	int rate = 1;

	Light *m_light = new Light();

	// message used for publishing actuator control value
	std_msgs::Float32 actuator_msg;

	ros::NodeHandle private_node_handle_("~");
	ros::Publisher actuator_pub = n.advertise<std_msgs::Float32>("Distance", 1);

	// Tell ROS how fast to run this node.
	ros::Rate r(rate);

	ROS_INFO("Start Light Node");

	// Main loop.
	while (n.ok())
	{
		// Run spin function at the beginning of the loop to acquire new data from ROS topics.
		ros::spinOnce();

		// publish required data
//		actuator_pub.publish(actuator_msg);

		m_light->LightBlink();


		// sleep the node for the 1/rate seconds
		r.sleep();
	}

	return 0;
}
