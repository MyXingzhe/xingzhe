#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "body.h"

const double UPDATE_RATE = 50; // desired publication rate of IMU data

int main(int argc, char **argv)
{
    
    // ROS set-ups:
    ros::init(argc, argv, "mpu6050"); // node name

    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

    ROS_INFO("main: instantiating an object of type ros_mpu6050");
    Body body(&nh);  // instantiate an ros_mpu6050 object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE);  // a timer for desired rate, 50Hz is a good speed. We set to half for 2 seperate sleeps

    ROS_INFO("Starting Data Recording From MPU6050");
    // loop to constantly "fetch" values from the MPU-6050
    while (ros::ok()) {
        body.Feeling();
        ros::spinOnce();
        sleep_timer.sleep();  // finish sleep time
    }
    return 0;
}
