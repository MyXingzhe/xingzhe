#!/bin/sh

g++ -o test_pru test_pru.cpp -l prussdrv

pasm -V3 -b pru0_test.p pru0
