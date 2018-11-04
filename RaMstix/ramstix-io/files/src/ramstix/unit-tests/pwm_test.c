/**
 * @file pwm_test.c
 * @brief PWM test with alternating duty cycle.
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>

#include "ramstix/ramstix_pwmout.h"

#define DELAY 1         // Start after 1 second
#define LOOP_FREQ 1     // Run at 1 Hz.
void* unitTest(void* ptr);
void exit_handler(int val);

int run_test;
int fd;

int main(int argc, char* argv[])
{
    // Set up a real-time xenomai thread with POSIX.
    mlockall( MCL_CURRENT | MCL_FUTURE );

    pthread_t thread;
    int rc, i;
    
    struct timespec rqtp, rmtp;

    struct sched_param sparam;
    sparam.sched_priority = 99;

    rqtp.tv_sec = 0;
    rqtp.tv_nsec = 1000000000/LOOP_FREQ;

    printf("\nPWM test\nConfigured: no braking VNH3SP30\n\n");
    // Setup signal handler for clean application exit.
    struct sigaction sa;
    sa.sa_handler = exit_handler;
    sigemptyset(&sa.sa_mask);
    if (-1 == sigaction(SIGINT, &sa, NULL))
    {
        printf("Error: Could not register exit signal\n");
        return 1;
    }
    run_test = 1;
    // init pwm
    fd = openGPMCFPGA();
    if (0 > fd)
    {
        printf("Error: Could not open GPMC device.\n");
        return 1;
    }
    for (i = 0; i < PWM_TOTAL_NUM; ++i)
    {
        ramstixSetPWMFrequency(fd, i, PWM_DEFAULT_FREQUENCY);
        ramstixInitPWMOut(fd, i, VNH3SP30, NO_BRAKING, NON_INTERLACED);
    }
    rc = pthread_create(&thread, NULL, unitTest, NULL);
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

    // uninitialise pwm:
    for (i = 0; i < PWM_TOTAL_NUM; ++i)
    {
        ramstixClosePWMOut(fd, i);
    }
    closeGPMCFPGA(fd);
    return 0;
}


void* unitTest(void* ptr)
{
    unsigned long overruns_r;
    int i;
    double count = 0.1, value = 0.05;
    while (run_test)
    {
        pthread_wait_np(&overruns_r);
        if (value > 0.5)
        {
            count = -0.1;
        }
        else if (value < -0.5)
        {
            count = 0.1;
        }
        value += count;
        printf("== Perform Test\n");
        for (i = 0; i < PWM_TOTAL_NUM; ++i)
        {
            printf("PWM %i: %f\n", i+1, value);
            ramstixSetPWMValue(fd, i, value);
        }
    }
    return 0;
}

void exit_handler(int val)
{
    printf("Exit from test\n");
    run_test = 0;
}
