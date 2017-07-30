.origin 0
.entrypoint START_PWM

#include "hm_pru.hp"



START_PWM:

    // Enable OCP master port
    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO    r0, CONST_PRUCFG, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
    // field to 0x0120.  This will make C28 point to 0x00012000 (PRU shared RAM).
    MOV     r0, 0x00000120
    MOV     r1, CTPPR_0
    MOV     r0, r1

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    MOV     r0, 0x00100000
    MOV     r1, CTPPR_1
    MOV     r0, r1

    MOV     r10, 0x22000
    LBBO    r11, r10, 0xC, 4 // read the cycle counter
    MOV     r12 , r11    // r12 stores start time of period

LOOP_POINT:
    
    LBCO    r0, CONST_DDR, 0, 4     // r0: switch flag of each pwm
    LBCO    r1, CONST_DDR, 4, 4     // r1: period of PWM, all pwms used the same period
    LBCO    r2, CONST_DDR, 8, 32   // r2~r9: duty of each pwm

    MOV     r10, 0x22000
    LBBO    r11, r10, 0xC, 4 // read the cycle counter into r11
    ADD     r21, r12, r1  // timeout of period


PWM_0:
    QBBC    PWM_1, r0, 0    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r2  // r20=expected timeout of pwm0 duty
    QBLE    PWM_0_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t0
PWM_0_PERIOD_TIMEOUT:
    QBLE    PWM_1, r11, r21
    SET     r30.t0

PWM_1:
    QBBC    PWM_2, r0, 1    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r3  // r20=expected timeout of pwm0 duty
    QBLE    PWM_1_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t1
PWM_1_PERIOD_TIMEOUT:
    QBLE    PWM_2, r11, r21
    SET     r30.t1

PWM_2:
    QBBC    PWM_3, r0, 2    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r4  // r20=expected timeout of pwm0 duty
    QBLE    PWM_2_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t2
PWM_2_PERIOD_TIMEOUT:
    QBLE    PWM_3, r11, r21
    SET     r30.t2

PWM_3:
    QBBC    PWM_3, r0, 3    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r5  // r20=expected timeout of pwm0 duty
    QBLE    PWM_3_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t3
PWM_3_PERIOD_TIMEOUT:
    QBLE    PWM_4, r11, r21
    SET     r30.t3

PWM_4:
    QBBC    PWM_5, r0, 4    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r6  // r20=expected timeout of pwm0 duty
    QBLE    PWM_4_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t4
PWM_4_PERIOD_TIMEOUT:
    QBLE    PWM_5, r11, r21
    SET     r30.t4

PWM_5:
    QBBC    PWM_2, r0, 5    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r7  // r20=expected timeout of pwm0 duty
    QBLE    PWM_5_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t5
PWM_5_PERIOD_TIMEOUT:
    QBLE    PWM_6, r11, r21
    SET     r30.t5

PWM_6:
    QBBC    PWM_2, r0, 6    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r8  // r20=expected timeout of pwm0 duty
    QBLE    PWM_6_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t6
PWM_6_PERIOD_TIMEOUT:
    QBLE    PWM_7, r11, r21
    SET     r30.t6

PWM_7:
    QBBC    PWM_2, r0, 7    // pwm_0 is not used
    // IF duty is timeout, set to low
    ADD r20, r12, r9  // r20=expected timeout of pwm0 duty
    QBLE    PWM_7_PERIOD_TIMEOUT, r11, r20
    CLR     r30.t7
PWM_7_PERIOD_TIMEOUT:
    MOV     r10, 0x22000
    LBBO    r11, r10, 0xC, 4 // reload current cycle count into r11
    QBLE    PWM_0, r11, r21
    SET     r30.t7


    MOV     r10, 0x22000
    LBBO    r11, r10, 0xC, 4 // read the cycle counter
    MOV     r12, r11    // r12 stores start time of period

    JMP LOOP_POINT

    // Halt the processor
    HALT

