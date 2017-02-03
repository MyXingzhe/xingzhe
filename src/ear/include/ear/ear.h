#ifndef __EAR_H__
#define __EAR_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "mraa.hpp"
#include "math.h"

#define USONIC_0_BUS   2
#define USONIC_0_ADDR  (0x70)

#define CMD_MULTI_DETECT_MIN    0x1
#define CMD_MULTI_DETECT_MAX    0x2f

#define CMD_DETECT_0_5_METER    0xb4
#define CMD_DETECT_0_11_METER   0xbc

// actually, this is ULTRA-SONIC SENSOR
class Ear {
public:
	Ear();
	~Ear();

private:
	int EarInit(int connector, int pin);

private:
	uint32_t  ObjDistance;

	int m_connector;
	int m_pin;

	int cnt;

	mraa::I2c *m_i2c;

public:
	uint32_t DoListen();

};


#endif  //__EAR_H__x`