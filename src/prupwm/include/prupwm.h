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
    uint32_t duty[6];
};

class PruPwm
{

public:
	PruPwm();
	~PruPwm();

	void Setup();
	void Close();
	void SetDuty(uint32_t channel, float duty);
	void SetPeriod(float period);
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
