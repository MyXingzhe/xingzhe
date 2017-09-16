#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"

#include "prupwm.h"

const double UPDATE_RATE = 50; // desired publication rate of IMU data

int main(int argc, char **argv)
{
    struct prupwm_param *pru;
    // ROS set-ups:
    ros::init(argc, argv, "mpu6050"); // node name

    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

    ROS_INFO("main: instantiating an object of type pru pwm");
    PruPwm *prupwm = new PruPwm();  // instantiate an ros_mpu6050 object and pass in pointer to nodehandle for constructor to use
    prupwm->Setup();
    ros::Rate sleep_timer(UPDATE_RATE);  // a timer for desired rate, 50Hz is a good speed. We set to half for 2 seperate sleeps

    ROS_INFO("Starting Data Recording From MPU6050");
    // loop to constantly "fetch" values from the MPU-6050
    while (ros::ok()) {
        ros::spinOnce();

        pru = prupwm->Report();
//        ROS_INFO("flag=0x%x, period=0x%x, cycle0=0x%x, cycle1=0x%x, cycle3=0x%x, cycle4=0x%x", pru->flag, pru->period, pru->cycle[0], pru->cycle[1], pru->cycle[2], pru->cycle[3]);

        sleep_timer.sleep();  // finish sleep time
    }
    return 0;
}
