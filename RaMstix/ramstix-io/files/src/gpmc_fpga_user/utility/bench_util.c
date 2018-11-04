#include <stdio.h>

#include <sys/mman.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // close()
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include "gpmc_fpga_user/gpmc_fpga_user.h"

#define FREQUENCY   1000        // frequency in Hz.
#define DELAY       1           // time before thread starts in seconds.
#define CALLS       1

void* toggleTask(void* ptr);
int g_calls;

int main(int argc, char* argv[])
{
    // Set up a real-time xenomai thread with POSIX.
    mlockall( MCL_CURRENT | MCL_FUTURE );

    pthread_t thread;
    int rc;
    struct timespec rqtp, rmtp;


    struct sched_param sparam;
    sparam.sched_priority = 99;

    if (argc >= 2)
    {
        // Usin

        char* ptr;
        long int freq = strtol(argv[1], &ptr, 10);
        printf("Using frequency: %li Hz\n", freq);
        rqtp.tv_sec = 0;
        rqtp.tv_nsec = 1000000000/freq; // Nano seconds for the frequency
    }
    else
    {
        printf("Using default frequency: 1000 Hz\n");
        rqtp.tv_sec = 0;
        rqtp.tv_nsec = 1000000000/FREQUENCY;

    }

    if (argc >= 3)
    {
        // Usin

        char* ptr;
        g_calls = strtol(argv[2], &ptr, 10);
        printf("Calls: %i Hz\n", g_calls);
    }
    else
    {
        g_calls = CALLS;
        printf("Calls: %i Hz\n", g_calls);

    }


    rc = pthread_create(&thread, NULL, toggleTask, NULL);
    assert(0 == rc);

    rc = pthread_setschedparam(thread, SCHED_FIFO, &sparam);
    assert(0 == rc);

    rc = clock_gettime(CLOCK_REALTIME, &rmtp );
    assert(0 == rc);
    rmtp.tv_sec = rmtp.tv_sec + DELAY;

    rc = pthread_make_periodic_np(thread, &rmtp, &rqtp);
    if(rc == ETIMEDOUT)
    {
        printf("TIMEOUT\n");
    }
    else if(rc == ESRCH)
    {
        printf("Invalid Thread \n");
    }
    assert(0 == rc);


    rc = pthread_join(thread, NULL);

    return 0;
}


void* toggleTask(void* ptr)
{
    int fd = -1, toggle = 0;
    unsigned long overruns_r;
    printf("opening gpmc device \n");

    int fd = openGPMCFPGA();
    if (0 > fd) {
	printf("Error opening device, check if kernel module is running.\n");
        return 1;
    }
    printf("Device opened. fd: %d \n", fd);

    printf("Starting loop, application will probably hang from now.\n");
    struct gpmc_fpga_data data;
    int ret, i;
    data.offset = 33;
    while (1)
    {
        ret = pthread_wait_np(&overruns_r);
       /* if (0 != ret)
        {
            printf("Ret: %s\n", strerror(ret));
            return 1;
        }*/
        for (i = 0; i < g_calls; ++i) {
            if (0 == toggle)
            {
                toggle = 0xffffffff;
            }
            else
            {
                toggle = 0;
            }
            data.data = toggle;

            ioctl(fd, IOCTL_SET_U16, &data);
            ioctl(fd, IOCTL_GET_U16, &data);
        }
        //printf("Toggle is %i\n", toggle);
        //sleep(1);
    }
    return 0;
}

