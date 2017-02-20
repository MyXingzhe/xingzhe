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
	uint16_t fifo_count;
	uint16_t packet_size;
	uint8_t fifo_buffer[64];

private:
	MPU6050 *m_imu;
	MPU6050 imu;
	mraa::Gpio *m_gpio;

	uint8_t dmpDataReady;
	bool m_ready;

	ros::NodeHandle nh_;
	ros::Publisher imu_publisher;
	ros::Publisher gpio_publisher;
	sensor_msgs::Imu data_out;  // variable name for our sensor_msgs::Imu output
	int16_t ax, ay, az, gx, gy, gz;  // temp variables to store data from imu.getMotion6(...)


	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


};


#endif  //__BODY_H__`