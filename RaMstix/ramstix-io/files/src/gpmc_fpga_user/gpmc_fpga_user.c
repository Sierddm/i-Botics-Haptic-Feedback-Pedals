/**
 * @file gpmc_fpga_user.c
 * @brief General GPMC driver functions implementation.
 * @author Jan Jaap Kempenaar, University of Twente
 * @author Wilbert van de Ridder, University of Twente
 * @author Marcel Schwirtz, University of Twente
 */
#include "gpmc_fpga_user/gpmc_fpga_user.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>

const char * array[] = {
    "/dev/",
    "/dev/rtdm/",
};

#define n_array (sizeof (array) / sizeof (const char *))

int openGPMCFPGA(void)
{
    int i,fd;
    char device[50];

    for(i = 0; i < n_array; i++) {
        memset(device,0,sizeof(device));
        strcat(device,array[i]);
        strcat(device,DEVICE_NAME_RT);

        fd = open(device, O_RDWR);

        if (0 > fd) {
            memset(device,0,sizeof(device));
            strcat(device,array[i]);
            strcat(device,DEVICE_NAME_NRT);

            fd = open(device, O_RDWR);
        }
        if (fd > 0) {
            printf("Found device %s\n",device);
            return fd;
        }
    }
    printf("NO DEVICE FOUND !!\n");
    return fd;
}

void closeGPMCFPGA(int fd)
{
    close(fd);
}

inline void writeGPMCFPGA32(int fd, int offset, int data)
{
    struct gpmc_fpga_data temp;
    temp.offset = offset;
    temp.data = data;
    ioctl(fd, IOCTL_SET_U32, &temp);
}

inline int readGPMCFPGA32(int fd, int offset)
{
    struct gpmc_fpga_data temp;
    temp.offset = offset;
    temp.data = 0;
    ioctl(fd, IOCTL_GET_U32, &temp);
    return temp.data;
}

inline void writeGPMCFPGA16(int fd, int offset, int data)
{
    struct gpmc_fpga_data temp;
    temp.offset = offset;
    temp.data = data;
    ioctl(fd, IOCTL_SET_U16, &temp);
}

inline int readGPMCFPGA16(int fd, int offset)
{
    struct gpmc_fpga_data temp;
    temp.offset = offset;
    temp.data = 0;
    ioctl(fd, IOCTL_GET_U16, &temp);
    return temp.data;
}

inline void readGPMCFPGA64(int fd, int offset, int* data)
{
    struct gpmc_fpga_data64 temp;
    temp.offset = offset;
    ioctl(fd, IOCTL_GET_U64, &temp);
    data[0] = temp.data.ints[0];
    data[1] = temp.data.ints[1];
}
