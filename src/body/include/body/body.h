#ifndef __BODY_H__
#define __BODY_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// message type used to communicate GPIO output

#include "MPU6050.h"
#include "mraa.hpp"

// actually, this is MPU6050 SENSOR


class Body
{

public:
	Body(ros::NodeHandle* nodehandle);
	~Body();

	void initializePublishers();
	void Feeling();
	int Setup();

private:

public:

private:
	MPU6050 *m_imu;
	MPU6050 mpu;

	bool m_ready;
	uint8_t m_status;
	uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifo_count;     // count of all bytes currently in FIFO
	uint8_t fifo_buffer[64]; // FIFO storage buffer

	ros::NodeHandle nh_;
	ros::Publisher imu_publisher;
	sensor_msgs::Imu imu_data;  // variable name for our sensor_msgs::Imu output

	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 accel;      // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


};


#endif  //__BODY_H__`