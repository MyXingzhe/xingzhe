#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

static void *sharedMem;
struct pru_pwm_param *pwm_param;

#define MS_TO_CYCLE(ms)    ((ms)*200000)


struct pru_pwm_param{
	uint32_t flag;
	uint32_t period;
	uint32_t duty[8];

	uint32_t cycle[8];
};

void init()
{
    int ret;
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    printf("\nINFO: Starting PRU.\r\n");
    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return -1;
    }

    /* map the DDR memory */
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return -1;
    }

    pwm_param = (struct pru_pwm_param *)(ddrMem + OFFSET_DDR);
    memset(pwm_param, 0, sizeof(struct pru_pwm_param));
    pwm_param->period = MS_TO_CYCLE(0.5);

    /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    /* Execute example on PRU */
    ret = prussdrv_exec_program (0, PRU_BIN_NAME);
    printf("\tINFO: Executing PRU. ret=%d\r\n", ret);

    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    pwm_param = (struct pru_pwm_param *) sharedMem;

}

void dump_pru_pwm()
{
	int i;
	printf("####################################\n");
	for(i=0;i<8;i++)
	{
		printf("PWM%d: %s, period=%d, duty=%d, cycle=%d\n", i, ((pwm_param->flag & (1<<i))?"USED":"UNUSED"), pwm_param->period, pwm_param->duty[i], pwm_param->cycle[i]);
	}
	printf("####################################\n");
}

void change()
{
	printf("Select your change:\n");
}

int main(int argc, char *argv[])
{
	char c;
	init();

	do {
		dump_pru_pwm();
		change();
		c=getchar();
		switch (c)
		{
			case '0':
				if((pwm_param->flag & 1) == 0)
					pwm_param->flag |= 1;
				else
					pwm_param->flag &= 0xfe;
				break;
			case '1':
				if((pwm_param->flag & 2) == 0)
					pwm_param->flag |= 2;
				else
					pwm_param->flag &= 0xfd;
				break;
			case '2':
				if((pwm_param->flag & 4) == 0)
					pwm_param->flag |= 4;
				else
					pwm_param->flag &= 0xfb;
				break;
			case '3':
				if((pwm_param->flag & 8) == 0)
					pwm_param->flag |= 8;
				else
					pwm_param->flag &= 0xf7;
				break;
			case '4':
				if((pwm_param->flag & 0x10) == 0)
					pwm_param->flag |= 0x10;
				else
					pwm_param->flag &= 0xef;
				break;
			case '5':
				if((pwm_param->flag & 0x20) == 0)
					pwm_param->flag |= 0x20;
				else
					pwm_param->flag &= 0xdf;
				break;
			case '6':
				if((pwm_param->flag & 0x40) == 0)
					pwm_param->flag |= 0x40;
				else
					pwm_param->flag &= 0xbf;
				break;
			case '7':
				if((pwm_param->flag & 0x80) == 0)
					pwm_param->flag |= 0x80;
				else
					pwm_param->flag &= 0x7f;
				break;
			case 'p':
				break;
			case 'q':
				goto quit;
				break;
		}
	}while(1);

quit:


	return 0;
}
