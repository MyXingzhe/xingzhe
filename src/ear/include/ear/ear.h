#ifndef __EAR_H__
#define __EAR_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "i2c_dev.h"
#include "i2c_func.h"

#define USONIC_0_ADDR  (0x70)

// actually, this is ULTRA-SONIC SENSOR
class Ear: public I2cFunc {
public:
	Ear(unsigned char bus, unsigned char addr);
	~Ear();

private:
	int EarInit(int connector, int pin);

private:
	uint32_t  ObjDistance;

	int m_connector;
	int m_pin;

public:
	uint32_t  DoListen();

};


#endif  //__EAR_H__x`