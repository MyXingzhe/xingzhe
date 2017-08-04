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

// Class derived from node-pru Node.js module
 
#ifndef __prupwm_h
#define __prupwm_h

// System headers
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string>
#include <stdint.h>

#define MS_TO_CYCLE(ms)    ((ms)*200000)

const uint32_t default_period =100;
const uint32_t default_duty  = 100;

/*  */

struct pru_pwm_param{
	uint32_t flag;
	uint32_t period;
	uint32_t duty[8];
};

class PRUPWM {
public:
	PRUPWM(uint8_t channel);
	~PRUPWM();

	void start();
	void stop();
	void set_period(uint32_t period);
	void set_duty(uint32_t duty);

private:
	uint32_t m_period;
	uint32_t m_duty;
	uint8_t m_channel;

	void *pruDataMem;
	struct pru_pwm_param *param;
};

#endif