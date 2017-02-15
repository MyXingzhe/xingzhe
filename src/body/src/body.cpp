
#include "body.h"

Body::Body(ros::NodeHandle* nodehandle)
     :nh_(*nodehandle)
{
	uint8_t status;

    ROS_INFO("in class constructor of Body");

//    initializePublishers();
    m_imu = new MPU6050();

    ROS_INFO("Initializing I2C devices...");
    m_imu->initialize();

    ROS_INFO("Initializing DMP...");
//    status = m_imu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    m_imu->setXGyroOffset(220);
    m_imu->setYGyroOffset(76);
    m_imu->setZGyroOffset(-85);
    m_imu->setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (status == 0) {
        // turn on the DMP, now that it's ready
        ROS_INFO("Enabling DMP...");
//        m_imu->setDMPEnabled(true);

//        attachInterrupt(0, dmpDataReady, RISING);
//        mpuIntStatus = m_imu->getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ROS_INFO("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
//        packetSize = m_imu->dmpGetFIFOPacketSize();
    } else {
    	dmpReady = false;
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        ROS_INFO("DMP Initialization failed (code =%d)", status);
    }
}

Body::~Body()
{

}

