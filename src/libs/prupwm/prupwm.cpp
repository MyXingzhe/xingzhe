/* Copyright (c) 2013 Owen McAree
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdint.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "prupwm.h"

PRUPWM::PRUPWM(uint8_t channel) {
	m_channel = channel;
	prussdrv_map_prumem (PRUSS0_PRU0_DATARAM, &pruDataMem);
	param = (struct pru_pwm_param *)pruDataMem;
}

PRUPWM::~PRUPWM()
{
//	munmap(pruDataMem);
}

void PRUPWM::start() {
	param->flag |= (m_channel << 1);
}

void PRUPWM::stop() {
	param->flag &= ~(m_channel << 1);
}

/* period is in micro seconds */
void PRUPWM::set_period(uint32_t period) {
	m_period = period;
	param->period = MS_TO_CYCLE(period);
}

void PRUPWM::set_duty(uint32_t duty) {
	m_duty = duty;
	param->duty[m_channel] = MS_TO_CYCLE(duty);
}


