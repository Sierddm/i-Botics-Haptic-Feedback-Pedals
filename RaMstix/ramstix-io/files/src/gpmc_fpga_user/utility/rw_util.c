#include <stdio.h>

#include <sys/mman.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // close()
#include "gpmc_fpga_user/gpmc_fpga_user.h"
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

    if (argc != 4 && argc != 5)
    {
        printf(USAGE, argv[0]);
        return 0;
    }

    int fd = openGPMCFPGA();
    if (0 > fd) {
	printf("Error opening device, check if kernel module is running.\n");
        return 1;
    }
    printf("Device opened. fd: %d \n", fd);

    char* ptr;
    int offset = strtol(argv[2], &ptr, 10);
    int action = *argv[3];
    int data_size = strtol(argv[1], &ptr, 10);
    int read, write;
    switch(data_size)
    {
        case 16:
            read = IOCTL_GET_U16;
            write = IOCTL_SET_U16;
            break;

        case 32:
            read = IOCTL_GET_U32;
            write = IOCTL_SET_U32;
            break;
        case 64:
            read = IOCTL_GET_U64;
            write = IOCTL_SET_U64;
            break;
        default:
            printf("Data size can only 16-bit or 32-bit.\n");
            return 1;
            break;
    }

    switch (action)
    {
        case 'w':
        case 'W':
            if (64 == data_size)
            {
                struct gpmc_fpga_data64 data;
                if (argc != 5)
                {
                    printf(USAGE, argv[0]);
                    return 1;
                }
                data.offset = offset;
                data.data.longval = strtol(argv[4], &ptr, READ_TYPE);
                printf("Writing(%i): %li, %lx\n", offset, data.data.longval, data.data.longval);
                ioctl(fd, write, &data);
            }
            else
            {
                struct gpmc_fpga_data data;
                if (argc != 5)
                {
                    printf(USAGE, argv[0]);
                    return 1;
                }
                data.offset = offset;
                data.data = strtol(argv[4], &ptr, READ_TYPE);
                printf("Writing(%i): %i, %x\n", offset, data.data, data.data);
                ioctl(fd, write, &data);
            }
            break;

        case 'r':
        case 'R':
            if (64 == data_size)
            {
                struct gpmc_fpga_data64 data;
                data.offset = offset;
                ioctl(fd, read, &data);
                printf("Reading(%i): %x, %x\n", offset, data.data.ints[0], data.data.ints[1]);
            }
            else
            {
                struct gpmc_fpga_data data;
                data.offset = offset;
                ioctl(fd, read, &data);
                printf("Reading(%i): %i, %x\n", offset, data.data, data.data);
            }
            break;

        default:
            printf("Unsupported action.\n");
            break;
    }
    printf("Closing\n");
    closeGPMCFPGA(fd);
    return 0;
}
