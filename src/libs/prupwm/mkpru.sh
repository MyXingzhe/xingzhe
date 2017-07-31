#!/bin/sh
pasm -b pwm.p pwm

gcc -o pru0 pru.cpp -l libprussdrv
