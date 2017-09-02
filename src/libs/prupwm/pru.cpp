/* Copyright (c) 2013 Owen McAree
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

// Driver header file
#include "prussdrv.h"
#include <pruss_intc_mapping.h>

/******************************************************************************
* Global Function Definitions                                                 *
******************************************************************************/

#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR       0x00001000
#define OFFSET_SHAREDRAM 2048       //equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

#define PRU_BIN_NAME  "/lib/firmware/pru0.bin"

#define MS_TO_CYCLE(ms)    ((ms)*200000)

int mem_fd;
static void *ddrMem;
struct pru_pwm_param *pwm_param;

struct pru_pwm_param{
    uint32_t flag;
    uint32_t period;
    uint32_t duty[8];

    uint32_t cycle[8];
};


int main (void)
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

    while(1)
    {
        sleep(10);
    }


    return(0);
}
