
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string>
#include <stdint.h>

#include "pwmss.h"

Pwmss::Pwmss(uint32_t pin) {
	m_pwm = new mraa::Pwm(pin);
}

void Pwmss::start() {
	m_pwm->enable (true);
}

void Pwmss::setFrequency(uint32_t frequency) {
	pwmFrequency = frequency;

	m_pwm->period_ms(1000/frequency);
}

void Pwmss::setChannelValue(uint32_t channel, unsigned long pwm_ns){
}

void Pwmss::setPRUDuty(uint32_t channel, unsigned long pwm_ns) {

}

