#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_BIN_NAME  "./pru0.bin"

#define MS_TO_CYCLE(ms)    ((ms)*2000000/1000)

#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR       0x00001000
#define OFFSET_SHAREDRAM 2048       //equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4


typedef unsigned int uint32_t;

struct prupwm_param{
    uint32_t flag;
    uint32_t period;
    uint32_t duty[6];
};

void dump_data_duty(struct prupwm_param *pwm_param)
{
    int i;

    printf("Duty FOR PRM PWM:\n");
    printf("==============================================\n");
    for(i=0;i<6;i++) {
        printf("%x08\n", pwm_param->duty[i]);
    }
    printf("==============================================\n");
}

void dump_data_flag(struct prupwm_param *pwm_param)
{
    int i;

    printf("switch FOR PRM PWM:\n");
    printf("============================================== %x, %x\n", pwm_param->flag, pwm_param->period);
    for(i=0;i<6;i++) {
        printf("PWM-%d: ", i);
        if(pwm_param->flag & (1 << i))
            printf("ON\n");
        else
            printf("OFF\n");
    }
    printf("==============================================\n");
}

void switch_flag(struct prupwm_param *pwm_param, int bit)
{
    if(pwm_param->flag & (1<<bit)) {
        pwm_param->flag &= (~(1<<bit));
    } else {
        pwm_param->flag |= (1<<bit);
    }
}

int
main(int argc, char const *argv[])
{
    int ret;
    int i;
    void *sharedMem;
    struct prupwm_param *pwm_param;

    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    printf("\nINFO: Starting PRU.\r\n");
    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return -1;
    }

     /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &sharedMem);

    pwm_param = (struct prupwm_param *)(sharedMem);
    memset(pwm_param, 0, sizeof(struct prupwm_param));
    pwm_param->flag = 0xcf;
    pwm_param->period = MS_TO_CYCLE(20);

    for(i=0;i<6;i++) {
        pwm_param->duty[i] = MS_TO_CYCLE(1.5);
    }

    /* Execute example on PRU */
    ret = prussdrv_exec_program (0, PRU_BIN_NAME);
    printf("\tINFO: Executing PRU. ret=%d\r\n", ret);

    while(1) {
        switch(getchar())
        {
            case 'd':
                if(pwm_param->duty[0] > 100)
                    pwm_param->duty[0] -= 100;
                else
                    pwm_param->duty[0] = 0;

                dump_data_duty(pwm_param);
                break;

            case 'u':
                if(pwm_param->duty[0] < pwm_param->period)
                    pwm_param->duty[0] += 100;
                else
                    pwm_param->duty[0] = pwm_param->period;

                dump_data_duty(pwm_param);
                break;

            case 'q':
                return -1;

            case '0':
                switch_flag(pwm_param, 0);
                dump_data_flag(pwm_param);
                break;
            case '1':
                switch_flag(pwm_param, 1);
                dump_data_flag(pwm_param);
                break;
            case '2':
                switch_flag(pwm_param, 2);
                dump_data_flag(pwm_param);
                break;
            case '3':
                switch_flag(pwm_param, 3);
                dump_data_flag(pwm_param);
                break;
            case '4':
                switch_flag(pwm_param, 4);
                dump_data_flag(pwm_param);
                break;
            case '5':
                switch_flag(pwm_param, 5);
                dump_data_flag(pwm_param);
                break;

            default:
                dump_data_flag(pwm_param);
                dump_data_duty(pwm_param);
                break;
        }
        usleep(100);
    }
    prussdrv_pru_disable(0); 
    prussdrv_exit ();

    return 0;
}
