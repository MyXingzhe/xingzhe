#ifndef __SERVO_MOTOR_H__
#define __SERVO_MOTOR_H__

#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "prupwm.h"

class ServoMotor
{
public:
	ServoMotor(uint32_t channel);
	~ServoMotor();

	int Init();
	int Start();
	int Stop();

private:

public:

private:
	uint32_t m_channel;
	uint32_t m_state;
	float angel;

};

#endif
