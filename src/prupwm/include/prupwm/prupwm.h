#ifndef __BODY_H__
#define __BODY_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>



struct prupwm_param{
    uint32_t flag;
    uint32_t period;
    uint32_t duty[8];

    uint32_t cycle[8];
};

class PruPwm
{

public:
	PruPwm();
	~PruPwm();

	void Setup();
	void Close();
	struct prupwm_param *Report();

private:
	int mem_fd;
	void *ddrMem;
	void *sharedMem;
	struct prupwm_param *pwm_param;

public:

private:

};


#endif  //__BODY_H__`
