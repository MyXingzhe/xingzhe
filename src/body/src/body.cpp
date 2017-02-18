
#include "body.h"
#include "mraa.hpp"

const unsigned int GPIO = 30;
const double UPDATE_RATE = 50; // desired publication rate of IMU data
const double limg = 1000;  // used to convert rotational accel to deg/s
const double lima = 2*9.8;  // used to convert linear accel to m/s^2
int8_t imuid=0;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

Body::Body(ros::NodeHandle* nodehandle)
     :nh_(*nodehandle)
{
    uint8_t status;
    
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

    status = imu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    imu.setXGyroOffset(220);
    imu.setYGyroOffset(76);
    imu.setZGyroOffset(-85);
    imu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if(status == 0) {
        ROS_INFO("Enable MPU6050 OK!");
        imu.setDMPEnabled(true); 
    }

    m_ready = true;
    // get expected DMP packet size for later comparison
    packet_size = imu.dmpGetFIFOPacketSize();

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

#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_REALACCEL
void Body::Feeling()
{
    // get current FIFO count
    fifo_count = imu.getFIFOCount();

    imu.getFIFOBytes(fifo_buffer, packet_size);

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    imu.dmpGetQuaternion(&q, fifo_buffer);
    imu.dmpGetEuler(euler, &q);
    ROS_INFO("euler %f:%f:%f\t", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    imu.dmpGetQuaternion(&q, fifo_buffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ROS_INFO("ypr %f:%f:%f\t", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    imu.dmpGetQuaternion(&q, fifo_buffer);
    imu.dmpGetAccel(&aa, fifo_buffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    ROS_INFO("areal %d:%d:%d\t", aaReal.x, aaReal.y, aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    imu.dmpGetQuaternion(&q, fifo_buffer);
    imu.dmpGetAccel(&aa, fifo_buffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    imu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    ROS_INFO("aworld  %d:%d:%d\t", aaWorld.x, aaWorld.y, aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifo_buffer[0];
    teapotPacket[3] = fifo_buffer[1];
    teapotPacket[4] = fifo_buffer[4];
    teapotPacket[5] = fifo_buffer[5];
    teapotPacket[6] = fifo_buffer[8];
    teapotPacket[7] = fifo_buffer[9];
    teapotPacket[8] = fifo_buffer[12];
    teapotPacket[9] = fifo_buffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

}
