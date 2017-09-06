#ifndef __BODY_H__
#define __BODY_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


class PruPwm
{

public:
	PruPwm(ros::NodeHandle* nodehandle);
	~PruPwm();

	void initializePublishers();
	int Setup();

private:

public:

private:
	bool m_ready;
	uint8_t m_status;
	uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifo_count;     // count of all bytes currently in FIFO
	uint8_t fifo_buffer[64]; // FIFO storage buffer

	ros::NodeHandle nh_;
	ros::Publisher imu_publisher;
};


#endif  //__BODY_H__`
