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
 
#ifndef prupwm_h
#define prupwm_h

// System headers
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string>

#include "pru.h"

class PRUPWM: PRU {
public:
	PRUPWM(uint32_t frequency);

	void start();
	void setFrequency(uint32_t frequency);
	void setChannelValue(uint32_t channel, unsigned long pwm_ns);
	void setFailsafeValue(uint32_t channel, unsigned long pwm_ns);
	void setFailsafeTimeout(uint32_t timeout_ms);
	void setPRUDuty(uint32_t channel, unsigned long pwm_ns);
	void updateFailsafe();
private:
	uint32_t pwmFrequency;
	uint32_t failsafeTimeout;
	const static uint32_t nanosecondsPerCycle = 5;
};

#endif