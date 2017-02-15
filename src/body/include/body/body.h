#ifndef __BODY_H__
#define __BODY_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "MPU6050.h"

// actually, this is MPU6050 SENSOR

class Body
{

public:
	Body(ros::NodeHandle* nodehandle);
	~Body();

	void initializePublishers();
	void fetchValues();

private:

public:

private:
	MPU6050 *m_imu;
	uint8_t mpuIntStatus;
	uint8_t dmpDataReady;
	bool dmpReady;

	ros::NodeHandle nh_;
	ros::Publisher imu_publisher;
	ros::Publisher gpio_publisher;
	sensor_msgs::Imu data_out;  // variable name for our sensor_msgs::Imu output
///	tsn_bbb_msgs::GPIOOut gpio_data_out;  // variable name for our gpio output
	int16_t ax, ay, az, gx, gy, gz;  // temp variables to store data from imu.getMotion6(...)

};


#endif  //__BODY_H__`