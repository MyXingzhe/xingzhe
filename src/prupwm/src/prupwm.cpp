#include <sys/mman.h>

// Driver header file

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "prupwm.h"
#include "mraa.hpp"

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL

const double UPDATE_RATE = 50; // desired publication rate of IMU data

#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR       0x00001000
#define OFFSET_SHAREDRAM 2048       //equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

#define PRU_BIN_NAME  "/lib/firmware/pru0.bin"

#define MS_TO_CYCLE(ms)    ((ms)*200000)

static int mem_fd;
static void *ddrMem, *sharedMem;

PruPwm::PruPwm()
{

}

PruPwm::~PruPwm()
{

}

void PruPwm::Setup()
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
        return ;
    }

     /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

   /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return ;
    }

    /* map the DDR memory */
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return ;
    }

    pwm_param = (struct prupwm_param *)(ddrMem + OFFSET_DDR);
    memset(pwm_param, 0, sizeof(struct prupwm_param));
    pwm_param->flag = 0xcf;
    pwm_param->period = MS_TO_CYCLE(0.5);

    /* Execute example on PRU */
    ret = prussdrv_exec_program (0, PRU_BIN_NAME);
    printf("\tINFO: Executing PRU. ret=%d\r\n", ret);

    usleep(100);

struct prupwm_param *param = Report();
    printf("flag=0x%x, period=0x%x, cycle0=0x%x, cycle1=0x%x, cycle3=0x%x, cycle4=0x%x\n", param->flag, param->period, param->cycle[0], param->cycle[1], param->cycle[2], param->cycle[3]);

    return ;
}

struct prupwm_param *PruPwm::Report()
{
	int i;

	struct prupwm_param *param;
    prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);
    param = (struct prupwm_param *) ((unsigned int*)sharedMem + OFFSET_SHAREDRAM);

    for(i=0;i<18;i++) {
    	printf("0x%x,", *((unsigned int*)sharedMem + OFFSET_SHAREDRAM + i*4));
    }
    printf("\n");

	return param;
}

void PruPwm::Close()
{
    munmap(ddrMem, 0x0FFFFFFF);
    close(mem_fd);
}
