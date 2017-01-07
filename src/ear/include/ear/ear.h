#ifndef __EAR_H__
#define __EAR_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "i2c_dev.h"
#include "i2c_func.h"
#include "gpio.h"


// actually, this is ULTRA-SONIC SENSOR
class Ear: public I2cFunc
{

public:
	Ear(int connector, int pin);
	~Ear();

private:
	int EarInit();

private:
	uint32_t  ObjDistance;

	int m_connector;
	int m_pin;

public:
	uint32_t  DoListen();

};


#endif  //__EAR_H__x`