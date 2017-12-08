#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_BIN_NAME  "/lib/firmware/pru0.bin"

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

int
main(int argc, char const *argv[])
{
    int ret;
    int i;
    int mem_fd;
    void *ddrMem;

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

    pwm_param = (struct prupwm_param *)(ddrMem + OFFSET_DDR);
    memset(pwm_param, 0, sizeof(struct prupwm_param));
    pwm_param->flag = 0xcf;
    pwm_param->period = MS_TO_CYCLE(20);

    for(i=0;i<6;i++) {
        pwm_param->duty[i] = MS_TO_CYCLE(1.5);
    }

    /* Execute example on PRU */
    ret = prussdrv_exec_program (0, PRU_BIN_NAME);
    printf("\tINFO: Executing PRU. ret=%d\r\n", ret);

    return 0;
}
