#ifndef __EAR_H__
#define __EAR_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "i2c_func.h"

#define USONIC_0_BUS   2
#define USONIC_0_ADDR  (0x70)

// actually, this is ULTRA-SONIC SENSOR
class Ear {
public:
	Ear();
	Ear(uint8_t bus, uint8_t addr);
	~Ear();

private:
	int EarInit(int connector, int pin);

private:
	uint32_t  ObjDistance;

	int m_connector;
	int m_pin;

	I2cFunc *m_i2c;

public:
	uint32_t DoListen();

};


#endif  //__EAR_H__x`