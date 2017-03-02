#ifndef __PWMSS_H__
#define __PWMSS_H__

#include <mraa.hpp>

class Pwmss {
public:
	Pwmss(uint32_t pin);
	void start();
	void setFrequency(uint32_t frequency);
	void setChannelValue(uint32_t channel, unsigned long pwm_ns);
	void setPRUDuty(uint32_t channel, unsigned long pwm_ns);

private:
	mraa::Pwm *m_pwm;
	uint32_t pwmFrequency;

};

#endif
