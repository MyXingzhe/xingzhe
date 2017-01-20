#include "ros/ros.h"

#include "ear.h"

Ear::Ear()
{
	
}

Ear::Ear(uint8_t bus, uint8_t addr)
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
	uint32_t dist_hi, dist_lo;
	m_i2c->I2cWriteByteData(USONIC_0_BUS, USONIC_0_ADDR, 2, 0xb4);

    dist_hi = I2cFunc::I2cReadByteData(USONIC_0_BUS, USONIC_0_ADDR, 2);
    if(dist_hi < 0)
        dist_hi = 0;

    dist_lo = I2cFunc::I2cReadByteData(USONIC_0_BUS, USONIC_0_ADDR, 3);
    if(dist_lo < 0)
        dist_lo = 0;

 	return dist_hi*255 + dist_lo;
}

int Ear::EarInit(int connector, int pin)
{
	m_connector = connector;
	m_pin = pin;
	
	return 0;
}