#include <stdio.h>

#include <sys/mman.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // close()
#include <stdlib.h>
#include "gpmc_fpga_user/gpmc_fpga_user.h"

int main(int argc, char* argv[])
{
#ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif

    if (argc != 3)
    {
        printf("Usage:\n%s <offset> <reg_count>\n", argv[0]);
        return 0;
    }

    int fd = openGPMCFPGA();
    if (0 > fd) {
	printf("Error opening device, check if kernel module is running.\n");
        return 1;
	}
    printf("Device opened. fd: %d \n", fd);


    char* ptr;
    int offset = strtol(argv[1], &ptr, 10);
    int reg_count = strtol(argv[2], &ptr, 10);
    int i;
    for (i = offset; i < (offset + reg_count); i++)
    {
        struct gpmc_fpga_data data;
        data.offset = i;
        ioctl(fd, IOCTL_GET_U16, &data);
        printf("Register %i:\t 0x%04x\n", i, 0xffff & data.data);
    }

    closeGPMCFPGA(fd);
    return 0;
}
