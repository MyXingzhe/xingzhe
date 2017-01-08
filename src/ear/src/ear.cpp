#include "ros/ros.h"

#include "ear.h"

Ear::Ear(unsigned char bus, unsigned char addr)
	:I2cFunc(bus, addr)
{
	ObjDistance = 0;
	m_connector = 0;
	m_pin = 0;

	ROS_DEBUG("Ear Constructor");
}

Ear::~Ear()
{

}

uint32_t Ear::DoListen()
{
	return 10;
}

int Ear::EarInit(int connector, int pin)
{
	m_connector = connector;
	m_pin = pin;
	
	return 0;
}