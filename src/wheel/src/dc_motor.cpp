#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "dc_motor.h"

DCMotor::DCMotor(uint32_t channel, uint32_t connector, uint32_t pin)
{
	m_channel = channel;
	m_dir = DIR_POSITIVE;
	m_connector = connector;
	m_pin = pin;
}

DCMotor::~DCMotor()
{

}

int DCMotor::Init()
{
	return 0;
}

int DCMotor::Start()
{
	return 0;
}

int DCMotor::Stop()
{
	return 0;
}

int DCMotor::SetDir(int dir)
{
	return 0;
}

int DCMotor::SetSpeed(uint32_t speed)
{
	return 0;
}

int DCMotor::SpeedUp()
{
	return 0;
}

int DCMotor::SpeedDown()
{
	return 0;
}
