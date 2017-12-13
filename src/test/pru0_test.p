.origin 0
.entrypoint START_PWM

#include "pru.hp"


// currently we support 6 pwms in pru0
//
//   PWM0     "P9.31",    /* pr1_pru0_pru_r30_0 */
//   PWM1     "P9.29",    /* pr1_pru0_pru_r30_1 */
//   PWM2     "P9.30",    /* pr1_pru0_pru_r30_2 */
//   PWM3     "P9.28",    /* pr1_pru0_pru_r30_3 */
//   PWM4     "P8.11",    /* pr1_pru0_pru_r30_15 */
//   PWM5     "P8.12",    /* pr1_pru0_pru_r30_14 */


.struct pwm_param
    .u32    flag
    .u32    period
    .u32    duty0
    .u32    duty1
    .u32    duty2
    .u32    duty3
    .u32    duty4
    .u32    duty5
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

    MOV     r28, PRU_ICSS_PRU0_CTRL
    LD32    r29, r28
    SET     r29, COUNTER_ENABLE_BIT
    ST32    r29, r28

    MOV     r28, PRUSS_CYCLE

LOOP_POINT:
    LBCO    r0, CONST_DDR, 0, 4

PWM_0:
    QBBC    PWM_1, r0.t0    // pwm_0 is not used
    SET     r30.t0
    CLR     r30.t0

PWM_1:
    QBBC    PWM_2, r0.t1    // pwm_0 is not used
    SET     r30.t1
    CLR     r30.t1

PWM_2:
    QBBC    PWM_3, r0.t2    // pwm_0 is not used
    SET     r30.t2
    CLR     r30.t2

PWM_3:
    QBBC    PWM_4, r0.t3    // pwm_0 is not used
    SET     r30.t3
    CLR     r30.t3

PWM_4:
    QBBC    PWM_5, r0.t4    // pwm_0 is not used
    SET     r30.t14
    CLR     r30.t14

PWM_5:
    QBBC    PERIOD_LOOP, r0.t5    // pwm_0 is not used
    SET     r30.t15
    CLR     r30.t15


    JMP LOOP_POINT

    // Halt the processor
    HALT

