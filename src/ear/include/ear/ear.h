#ifndef __EAR_H__
#define __EAR_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"


// actually, this is ULTRA-SONIC SENSOR
class Ear
{

public:
	Ear();
	~Ear();

private:
	uint32_t  ObjDistance;


public:
	uint32_t  DoListen();

};


#endif  //__EAR_H__x`