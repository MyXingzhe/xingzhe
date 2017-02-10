#ifndef __LIGHT_H__
#define __LIGHT_H__

#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "mraa.hpp"

#define DEFAULT_LIGHT_CONN  8
#define DEFAULT_LIGHT_PIN   7

enum LIGHT_STATE {
	LIGHT_OFF = 0,
	LIGHT_ON
};

class Light {
public:
	Light();
	Light(int conn, int pin);
	~Light();

	void LightOn();
	void LightOff();

	void LightBlink();

private:
	int LightInit();

private:
	int m_pin;
	int m_conn;
	int m_state;

	mraa::Gpio *m_gpio;

};


#endif
