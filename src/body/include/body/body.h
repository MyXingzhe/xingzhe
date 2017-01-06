#ifndef __BODY_H__
#define __BODY_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"

// actually, this is MPU9150 SENSOR

class Body
{

public:
	Body();
	~Body();

private:
	uint32_t  ObjDistance;

public:
	uint32_t  DoListen();

};


#endif  //__BODY_H__`