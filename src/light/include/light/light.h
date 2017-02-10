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

class Light {
public:
	Light();
	Light(int conn, int pin);
	~Light();

	void LightOn();
	void LightOff();

private:
	int LightInit();

private:
	int m_pin;
	int m_conn;

	mraa::Gpio *m_gpio;

};


#endif
