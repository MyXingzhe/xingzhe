#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

static void *pruDataMem;
struct pru_pwm_param *pwm_param;

#define MS_TO_CYCLE(ms)    ((ms)*200000)


struct pru_pwm_param{
	uint32_t flag;
	uint32_t period;
	uint32_t duty[8];
};

void init()
{
	prussdrv_map_prumem (PRUSS0_PRU0_DATARAM, &pruDataMem);
	pwm_param = (struct pru_pwm_param *)pruDataMem;
	pwm_param->period = MS_TO_CYCLE(0.5);
}

void dump_pru_pwm()
{
	int i;
	printf("####################################\n");
	for(i=0;i<8;i++)
	{
		printf("PWM%d: %s, period=%d, duty=%d\n", i, ((pwm_param->flag & (1<<i))?"USED":"UNUSED"), pwm_param->period, pwm_param->duty[i]);
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
			case 'q':
				goto quit;
				break;
		}
	}while(1);

quit:

	free(pruDataMem);

	return 0;
}
