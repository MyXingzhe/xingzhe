#!/bin/sh
pasm -b pwm.p pwm

gcc -o pru0 pru.cpp -l prussdrv
gcc -o prupwm-tester prupwm_test.cpp -l prussdrv
