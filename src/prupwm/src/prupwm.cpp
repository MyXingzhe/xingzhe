#include "prupwm.h"
#include "mraa.hpp"

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL

const double UPDATE_RATE = 50; // desired publication rate of IMU data


PruPwm::PruPwm(ros::NodeHandle* nodehandle)
     :nh_(*nodehandle)
{
}

PruPwm::~PruPwm()
{

}

int PruPwm::Setup()
{

    return 0;
}


// Simple function to set up all publishers (more can be added as desired)
void PruPwm::initializePublishers()  {
    ROS_INFO("Initializing Publishers: imu_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
}

