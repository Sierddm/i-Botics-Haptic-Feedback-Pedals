#include <stdio.h>

#include "ramstix/ramstix_qcntr.h"
#include <sys/mman.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // close()
/*#include "gpmc_fpga_user/gpmc_fpga_user.h"*/
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

#ifdef HEXADECIMAL
#define USAGE "Usage:\n%s <16|32|64> <offset> <R|W> [hex_value]\n"
#define READ_TYPE   16
#else
#define USAGE "Usage:\n%s <16|32|64> <offset> <R|W> [decimal_value]\n"
#define READ_TYPE   10
#endif

int main(int argc, char* argv[])
{
#ifdef __COBALT__
    printf("Xeno COBALT active!\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif

    int fd = openGPMCFPGA();
    if (0 > fd) {
	printf("Error opening device, check if kernel module is running.\n");
        return 1;
    }
    printf("Device opened. fd: %d \n", fd);
    
    // Configure encoder 1 as Quadrature encoder with reset on positive index.
    (fd, 1, POSEDGE_RESET, QUADRATURE);
    // Reset encoder to zero.
    ramstixSetQCounterValue(fd, 1, 0);
    // Read values from the encoder.
    int counter = ramstixGetQCounterValue(fd, 1);
    // result:
    printf("Counter value is: %i\n", counter);
    // close encoder counter and rt_device.
    ramstixCloseQCounter(fd, 1);

    printf("Closing\n");
    closeGPMCFPGA(fd);
    return 0;
}
