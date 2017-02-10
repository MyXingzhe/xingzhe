#include <limits>
#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "mraa.hpp"
#include "light.h"

Light::Light()
{
	m_conn = DEFAULT_LIGHT_CONN;
	m_pin = DEFAULT_LIGHT_PIN;
	m_state = LIGHT_OFF;

	LightInit();
	LightOff();
}

Light::Light(int conn, int pin)
{
	m_conn = conn;
	m_pin  = pin;
	m_state = LIGHT_OFF;

	LightInit();
	LightOff();
}

Light::~Light()
{
	delete m_gpio;
}

void Light::LightOn()
{
	m_gpio->write(1);
}

void Light::LightOff()
{
	m_gpio->write(0);
}

void Light::LightBlink()
{
	m_gpio->write(m_state);

	m_state = (m_state == LIGHT_OFF)?LIGHT_ON:LIGHT_OFF;
}

int Light::LightInit()
{
	int iopin = (m_conn - 8) * 44 + m_pin;
	m_gpio = new mraa::Gpio(iopin);
    if (m_gpio == NULL) {
        return mraa::ERROR_UNSPECIFIED;
    }

	mraa::Result response = m_gpio->dir(mraa::DIR_OUT);
    if (response != mraa::SUCCESS) {
        mraa::printError(response);
        return -1;
    }
}
