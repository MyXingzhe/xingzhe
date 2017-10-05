#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "prupwm_duty.h"
#include "prupwm_period.h"

#include "prupwm.h"

const double UPDATE_RATE = 50; // desired publication rate of IMU data

PruPwm *prupwm;
void SetDuty(prupwm::prupwm_duty::Request &Req)
{
    prupwm->SetPwmDuty(Req.duty);
}

void SetPeriod(prupwm::prupwm_period::Request &Req)
{
    prupwm->SetPeriod(Req.period);
}

int main(int argc, char **argv)
{
    struct prupwm_param *pru;
    // ROS set-ups:
    ros::init(argc, argv, "mpu6050"); // node name

    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

    ROS_INFO("main;: instantiating an object of type pru pwm");
    prupwm = new PruPwm();  // instantiate an ros_mpu6050 object and pass in pointer to nodehandle for constructor to use
    prupwm->Setup();
    ros::Rate sleep_timer(UPDATE_RATE);  // a timer for desired rate, 50Hz is a good speed. We set to half for 2 seperate sleeps

    ros::ServiceServer service_duty = nh.advertiseService("SetDuty", SetDuty);
    ros::ServiceServer service_period = nh.advertiseService("SetPeriod", SetPeriod);

    // loop to constantly "fetch" values from the MPU-6050
    while (ros::ok()) {
        ros::spinOnce();

        pru = prupwm->Report();
/*        ROS_INFO("flag=0x%x, period=0x%x, duty0=%d, duty1=%d, duty2=%d, \
            duty3=%d, duty4=%d, duty5=%d, duty6=%d, duty7=%d, cycle0=0x%x, \
            cycle1=0x%x, cycle2=0x%x, cycle3=0x%x, cycle4=0x%x, \
            cycle5=0x%x, cycle6=0x%x, cycle7=0x%x", 
            pru->flag, pru->period, pru->duty[0], pru->duty[1], pru->duty[2], pru->duty[3], pru->duty[4], pru->duty[5], pru->duty[6], pru->duty[7],
            pru->cycle[0], pru->cycle[1], pru->cycle[2], pru->cycle[3], pru->cycle[4], pru->cycle[5], pru->cycle[6], pru->cycle[7]);
*/
        sleep_timer.sleep();  // finish sleep time
    }
    return 0;
}
