#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "prupwm.h"
#include "servo_motor.h"


ServoMotor::ServoMotor(uint32_t channel)
{
	m_channel = channel;
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
