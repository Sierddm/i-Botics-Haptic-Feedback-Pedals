#include <stdio.h>

#include <sys/mman.h>
#include <fcntl.h>      // open()
#include <unistd.h>     // close()
#include <pthread.h>
#include <assert.h>
#include <stdlib.h>
#include "gpmc_fpga_user/gpmc_fpga_user.h"

#define FREQUENCY   1000        // frequency in Hz.
#define DELAY       1           // time before thread starts in seconds.
#define CALLS       1

#include <sys/timerfd.h>
#include <unistd.h>

struct periodic_info
{
    int timer_fd;
    unsigned long long wakeups_missed;
} p_str;

int make_periodic (unsigned int period, struct periodic_info *info);
void wait_period (struct periodic_info *info);
void* toggleTask(void* ptr);
int g_calls;
int per, fd;

int main(int argc, char* argv[])
{
    pthread_t thread;
    int rc;

    struct sched_param sparam;
    sparam.sched_priority = 99;
    if (argc < 2)
    {
        printf("Usage: %s <device> [frequency] [iocalls]\n", argv[0]);
        return 1;
    }
    fd = open(argv[1], 0);
    if (0 > fd)
    {
       printf("Failed opening.\n");
       return 1;
    }
    if (argc >= 3)
    {
        // Usin

        char* ptr;
        long int freq = strtol(argv[2], &ptr, 10);
        printf("Using frequency: %li Hz\n", freq);
        per = 1000000 / freq;
    }
    else
    {
        printf("Using default frequency: 1000 Hz\n");
        per = 1000000 / FREQUENCY;

    }

    if (argc >= 4)
    {
        char* ptr;
        g_calls = strtol(argv[3], &ptr, 10);
        printf("Calls: %i Hz\n", g_calls);
    }
    else
    {
        g_calls = CALLS;
        printf("Calls: %i Hz\n", g_calls);

    }


    rc = pthread_create(&thread, NULL, toggleTask, argv);
    assert(0 == rc);

    rc = pthread_setschedparam(thread, SCHED_FIFO, &sparam);
    assert(0 == rc);

    rc = pthread_join(thread, NULL);

    return 0;
}


void* toggleTask(void* ptr)
{
    int toggle = 0;
    struct periodic_info ;
    struct gpmc_fpga_data data;
    int i;
    data.offset = 33;
    make_periodic(per, &p_str);
    printf("Starting loop, application will probably hang from now.\n");
    while (1)
    {
        wait_period(&p_str);
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


int make_periodic(unsigned int period, struct periodic_info *info)
{
    int ret;
    unsigned int ns;
    unsigned int sec;
    int fd;
    struct itimerspec itval;

    /* Create the timer */
    fd = timerfd_create (CLOCK_MONOTONIC, 0);
    info->wakeups_missed = 0;
    info->timer_fd = fd;
    if (fd == -1)
        return fd;

    /* Make the timer periodic */
    sec = period/1000000;
    ns = (period - (sec * 1000000)) * 1000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;
    ret = timerfd_settime (fd, 0, &itval, NULL);
    return ret;
}

void wait_period(struct periodic_info *info)
{
    unsigned long long missed;
    int ret;

    /* Wait for the next timer event. If we have missed any the
       number is written to "missed" */
    ret = read (info->timer_fd, &missed, sizeof (missed));
    if (ret == -1)
    {
        perror ("read timer");
        return;
    }

    /* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
    if (missed > 0)
        info->wakeups_missed += (missed - 1);
}
