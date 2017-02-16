
#include "body.h"
#include "mraa.hpp"

const unsigned int GPIO = 30;
const double UPDATE_RATE = 50; // desired publication rate of IMU data
const double limg = 1000;  // used to convert rotational accel to deg/s
const double lima = 2*9.8;  // used to convert linear accel to m/s^2
int8_t imuid=0;

Body::Body(ros::NodeHandle* nodehandle)
     :nh_(*nodehandle)
{
    
    ROS_INFO("in class constructor of Body");

    initializePublishers();

    ROS_INFO("Starting MPU6050");
    // Type 0 to use the default address value for the MPU-6050 (Typically 68)
    if (imuid > 0)
        MPU6050 imu(imuid);
    else
        MPU6050 imu();

    ROS_INFO("Initializing MPU6050...");
    // Initialize the IMU setting the clock source, gyro-scale, accel-scale, sample rate 
    // (sample rate is 8kHz divided by # in setRate function -- see MPU6050.cpp), disable 
    // sleep mode, and sleep for 1 second at start-up
    imu.initialize();

    // Using GPIO to rise whenever we fetch I2C information

    ROS_INFO("Exporting and setting direction of GPIO.");

    GpioInit();

    ROS_INFO("Done Initializing!"); 
}

Body::~Body()
{

}

int Body::GpioInit()
{
    uint8_t value;
    m_gpio = new mraa::Gpio(GPIO);
    m_gpio->useMmap(true);
    mraa::Result response = m_gpio->dir(mraa::DIR_OUT_LOW);
    if (response != mraa::SUCCESS) {
        mraa::printError(response);
        return -1;
    }

    value = m_gpio->read();  // Verifying GPIO was set correctly

    if (value == 0)
        ROS_INFO("GPIO is set Low, success!");
    else
        ROS_WARN("GPIO is set High, error!");

    return 0;    
}


// Simple function to set up all publishers (more can be added as desired)
void Body::initializePublishers()  {
    ROS_INFO("Initializing Publishers: imu_publisher, gpio_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
    gpio_publisher = nh_.advertise<body::mpu6050_gpio>("gpio_out", 1, true);  // publish current GPIO value
}

// retrieve IMU angular and linear accel values from IMU registers
void Body::fetchValues()  {
    // Retrieve IMU register gyro/accel data
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data_out.header.stamp = ros::Time::now();  // time stamp the measurement

    data_out.linear_acceleration.x = (double)((ax+32767)*2*lima)/65534-lima;  // acc
    data_out.linear_acceleration.y = (double)((ay+32767)*2*lima)/65534-lima;
    data_out.linear_acceleration.z = (double)((az+32767)*2*lima)/65534-lima;

    data_out.angular_velocity.x = (double)((gx+32767)*2*limg)/65534-limg;  // gyro
    data_out.angular_velocity.y = (double)((gy+32767)*2*limg)/65534-limg;
    data_out.angular_velocity.z = (double)((gz+32767)*2*limg)/65534-limg;

    data_out.angular_velocity.x = data_out.angular_velocity.x*3.1415926/180;  // change to rad/s
    data_out.angular_velocity.y = data_out.angular_velocity.y*3.1415926/180;
    data_out.angular_velocity.z = data_out.angular_velocity.z*3.1415926/180;
  
    imu_publisher.publish(data_out);  // publish
}

// set gpio high and publish 1
void Body::setGPIOHigh()  {
    m_gpio->write(1);
    gpio_data_out.header.stamp = ros::Time::now();  // time stamp the measurement
    gpio_data_out.data = 1;
    gpio_publisher.publish(gpio_data_out);
}

// set gpio low and publish 0
void Body::setGPIOLow()  {
    m_gpio->write(0);
    gpio_data_out.header.stamp = ros::Time::now();  // time stamp the measurement
    gpio_data_out.data = 0;
    gpio_publisher.publish(gpio_data_out);
}

