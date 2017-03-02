#ifndef __SERVO_MOTOR_H__
#define __SERVO_MOTOR_H__

#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "prupwm.h"
#include "pwmss.h"

class ServoMotor
{
public:
	ServoMotor(uint32_t channel, uint32_t mode);
	~ServoMotor();

	int Init();
	int Start();
	int Stop();
	int RotateTo(float ang);
	int RotateAng(float ang, uint32_t dir);
	float GetAngle() {return m_angle;}

private:

public:

private:
	uint32_t m_channel;
	uint32_t m_state;
	float m_angle;

	uint32_t m_mode;

	Pwmss *m_pwm;

};

#endif
