.origin 0
.entrypoint START_PWM

#include "pru.hp"


.struct pwm_param
    .u32    flag
    .u32    period
    .u32    duty0
    .u32    duty1
    .u32    duty2
    .u32    duty3
    .u32    duty4
    .u32    duty5
    .u32    duty6
    .u32    duty7
    .u32    cycle0
    .u32    cycle1
    .u32    cycle2
    .u32    cycle3
    .u32    cycle4
    .u32    cycle5
    .u32    cycle6
    .u32    cycle7
.ends


START_PWM:

    // Enable OCP master port
    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO    r0, CONST_PRUCFG, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
    // field to 0x0120.  This will make C28 point to 0x00012000 (PRU shared RAM).
    MOV     r0, 0x00000120
    MOV     r1, CTPPR_0
    ST32    r0, r1

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    MOV     r0, 0x00100000
    MOV     r1, CTPPR_1
    ST32    r0, r1

    MOV     r10, 0x22000
    LBBO    r11, r10, 0xC, 4 // read the cycle counter

LOOP_POINT:
    
    LBCO    r0, CONST_DDR, 0, SIZE(pwm_param)
    .assign pwm_param, r0, r9, param

    SBCO    r0, CONST_PRUSHAREDRAM, 0, 40

    LBCO    r29, C26, IEP_GLOBAL_CFG, 4
    set     r29.t0
    SBCO    r29, C26, IEP_GLOBAL_CFG, 4

    MOV     r29, 1
    SBCO    r29, C26, IEP_COUNT, 4

PERIOD_LOOP:
    LBCO    r29, C26, IEP_COUNT, 4

    QBLE    PERIOD_TIMEOUT, r29, param.period

PWM_0:
    QBBC    PWM_1, r0.t0    // pwm_0 is not used
    QBLE    PWM_0_DUTY_TIMEOUT, r29, param.duty0
    SET     r30.t0
    JMP     PWM_1
PWM_0_DUTY_TIMEOUT:
    CLR     r30.t0

PWM_1:
    QBBC    PWM_2, r0.t1    // pwm_0 is not used
    QBLE    PWM_1_DUTY_TIMEOUT, r29, param.duty1
    SET     r30.t1
    JMP     PWM_2
PWM_1_DUTY_TIMEOUT:
    CLR     r30.t1

PWM_2:
    QBBC    PWM_3, r0.t2    // pwm_0 is not used
    QBLE    PWM_2_DUTY_TIMEOUT, r29, param.duty2
    SET     r30.t2
    JMP     PWM_3
PWM_2_DUTY_TIMEOUT:
    CLR     r30.t2

PWM_3:
    QBBC    PWM_4, r0.t3    // pwm_0 is not used
    QBLE    PWM_3_DUTY_TIMEOUT, r29, param.duty3
    SET     r30.t3
    JMP     PWM_4
PWM_3_DUTY_TIMEOUT:
    CLR     r30.t3

PWM_4:
    QBBC    PWM_5, r0.t4    // pwm_0 is not used
    QBLE    PWM_4_DUTY_TIMEOUT, r29, param.duty4
    SET     r30.t4
    JMP     PWM_5
PWM_4_DUTY_TIMEOUT:
    CLR     r30.t4

PWM_5:
    QBBC    PWM_6, r0.t5    // pwm_0 is not used
    QBLE    PWM_5_DUTY_TIMEOUT, r29, param.duty5
    SET     r30.t5
    JMP     PWM_6
PWM_5_DUTY_TIMEOUT:
    CLR     r30.t5

PWM_6:
    QBBC    PWM_7, r0.t6    // pwm_0 is not used
    QBLE    PWM_6_DUTY_TIMEOUT, r29, param.duty6
    SET     r30.t6
    JMP     PWM_7
PWM_6_DUTY_TIMEOUT:
    CLR     r30.t6

PWM_7:
    QBBC    PERIOD_LOOP, r0.t7    // pwm_0 is not used
    QBLE    PWM_7_DUTY_TIMEOUT, r29, param.duty7
    SET     r30.t7
    JMP     PERIOD_LOOP
PWM_7_DUTY_TIMEOUT:
    CLR     r30.t7


PERIOD_TIMEOUT:
PERIOD_TIMEOUT_PWM_0:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t0    // pwm_0 is not used
    SET     r30.t0
PERIOD_TIMEOUT_PWM_1:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t1    // pwm_0 is not used
    SET     r30.t1
PERIOD_TIMEOUT_PWM_2:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t2    // pwm_0 is not used
    SET     r30.t2
PERIOD_TIMEOUT_PWM_3:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t3    // pwm_0 is not used
    SET     r30.t3
PERIOD_TIMEOUT_PWM_4:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t4    // pwm_0 is not used
    SET     r30.t4
PERIOD_TIMEOUT_PWM_5:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t5    // pwm_0 is not used
    SET     r30.t5
PERIOD_TIMEOUT_PWM_6:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t6    // pwm_0 is not used
    SET     r30.t6
PERIOD_TIMEOUT_PWM_7:
    QBBC    PERIOD_TIMEOUT_PWM_1, r0.t7    // pwm_0 is not used
    SET     r30.t7
    
    JMP LOOP_POINT

    // Halt the processor
    HALT

