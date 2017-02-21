#include "body.h"
#include "mraa.hpp"

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL

const double UPDATE_RATE = 50; // desired publication rate of IMU data

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU


Body::Body(ros::NodeHandle* nodehandle)
     :nh_(*nodehandle)
{
}

Body::~Body()
{

}

int Body::Setup()
{
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    m_status = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (m_status == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        m_ready = true;

        // get expected DMP packet size for later comparison
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", m_status);
    }

    return 0;
}


// Simple function to set up all publishers (more can be added as desired)
void Body::initializePublishers()  {
    ROS_INFO("Initializing Publishers: imu_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
}

void Body::Feeling()
{
    // if programming failed, don't try to do anything
    if (!m_ready) return;

    // get current FIFO count
    fifo_count = mpu.getFIFOCount();

    if (fifo_count == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        return;
    } else if (fifo_count >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifo_buffer, packet_size);
        imu_data.header.stamp = ros::Time::now();  // time stamp the measurement
        
    #ifdef OUTPUT_READABLE_QUATERNION
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        imu_data.orientation.w = q.w;
        imu_data.orientation.x = q.x;
        imu_data.orientation.y = q.y;
        imu_data.orientation.z = q.z;
    #endif

    #ifdef OUTPUT_READABLE_EULER
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetEuler(euler, &q);
        imu_data.angular_velocity.x = euler[0];
        imu_data.angular_velocity.y = euler[1];
        imu_data.angular_velocity.z = euler[2];
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetAccel(&accel, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &accel, &gravity);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetAccel(&accel, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    #endif
ROS_INFO("%f:%f:%f:%f", imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
        imu_publisher.publish(imu_data);
    }
}
