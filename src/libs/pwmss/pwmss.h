#ifndef __PWMSS_H__
#define __PWMSS_H__

#include <mraa.hpp>


/* pwmss 0 eCAP0 <--> pwmchip0 */
/* pwmss 0 pwm0  <--> pwmchip2 */
/* pwmss 1 pwm1  <--> pwmchip4 */
/* pwmss 2 eCAP0 <--> pwmchip1 */
/* pwmss 2 pwm0  <--> pwmchip6 */

class Pwmss {
public:
	Pwmss(uint32_t pin, int chipid);
	void start();
	void setFrequency(uint32_t frequency);
	void setChannelValue(uint32_t channel, unsigned long pwm_ns);
	void setPRUDuty(uint32_t channel, unsigned long pwm_ns);

public:
	
private:
	mraa::Pwm *m_pwm;
	uint32_t pwmFrequency;

};

#endif
