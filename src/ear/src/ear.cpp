#include "ros/ros.h"

#include "ear.h"
#include "gpio.h"

Ear::Ear(int connector, int pin)
{
	ObjDistance = 0;
	m_connector = connector;
	m_pin = pin;

	ROS_DEBUG("Ear Constructor");
}

Ear::~Ear()
{

}

uint32_t Ear::DoListen()
{
	return 10;
}

int Ear::EarInit()
{
	
	return 0;
}