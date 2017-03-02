#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "prupwm.h"
#include "servo_motor.h"


ServoMotor::ServoMotor(uint32_t channel, uint32_t mode)
{
	m_channel = channel;

	m_mode = mode;
}

ServoMotor::~ServoMotor()
{

}

int ServoMotor::Init()
{
	return 0;
}

int ServoMotor::Start()
{
	return 0;
}

int ServoMotor::Stop()
{
	return 0;
}

int ServoMotor::RotateTo(float ang)
{
	m_angle = ang;

	return 0;
}

int ServoMotor::RotateAng(float ang, uint32_t dir)
{
	return 0;
}
