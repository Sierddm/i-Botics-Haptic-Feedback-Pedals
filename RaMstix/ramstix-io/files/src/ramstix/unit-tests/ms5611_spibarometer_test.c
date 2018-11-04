/**
 * @file ms5611_spibaromteer_test.c
 * @brief Test for the MS5611 barometer
 * @author Geert Folkertsma, University of Twente
 */

#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>

#include "ramstix/ramstix_ms5611_spibarometer.h"

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
    int rc;
    
    struct timespec rqtp, rmtp;

    struct sched_param sparam;
    sparam.sched_priority = 99;

    rqtp.tv_sec = 0;
    rqtp.tv_nsec = 1000000000/LOOP_FREQ;

    printf("Unit-test Skeleton, test will start after 1 second.\n");
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
    fd = openGPMCFPGA();
    
    if (0 > fd)
    {
        printf("Error: Could not open GPMC device.\n");
        return 1;
    }
    
    // Init the barometer
    ramstixInitMS5611(fd, 3); //CS 3 is the one right after the 2 SPI output pins
    
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
	ramstixCloseMS5611(fd, 3);
    return 0;
}


void* unitTest(void* ptr)
{
    unsigned long overruns_r;
    double pressure;
    while (run_test)
    {
        pthread_wait_np(&overruns_r);
        printf("Perform Test\n");
        
        pressure = ramstixGetMS5611Pressure(fd, 3);
        printf("[MS5611_barometer] Pressure: %f Pa\n",pressure);
    }
    return 0;
}

void exit_handler(int val)
{
    printf("Exit from test\n");
    run_test = 0;
}
