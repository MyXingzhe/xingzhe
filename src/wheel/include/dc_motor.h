#ifndef __DCMOTOR_H__
#define __DCMOTOR_H__

#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "prupwm.h"

enum {
	DIR_POSITIVE = 0,
	DIR_NEGATIVE
};

class DCMotor
{
public:
	DCMotor(uint32_t channel, uint32_t connector, uint32_t pin);
	~DCMotor();

	int Init();
	int Start();
	int Stop();
	int SetDir(int dir);
	int SetSpeed(uint32_t speed);
	int SpeedUp();
	int SpeedDown();

private:

public:

private:
	uint32_t m_channel;
	uint32_t m_dir;
	uint32_t m_state;
	uint32_t m_speed;  /* SPEED: milimeter in second */
	uint32_t m_connector;
	uint32_t m_pin;

};

#endif
