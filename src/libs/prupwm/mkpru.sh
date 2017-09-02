#!/bin/sh
pasm -b pwm.p pru0

gcc -o pru0 pru.cpp -l prussdrv
gcc -o prupwm-tester prupwm_test.cpp -l prussdrv
