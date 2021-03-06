/**
 * @file encoder_test.c
 * @brief Encoder test: Quadrature, noindex
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>
#include <time.h>

#include "ramstix/ramstix_qcntr.h"
#include "ramstix/ramstix_analog.h"

void unitTest();
void exit_handler(int val);

int fd;
int run_test = 1;
const int NUM_SECONDS = 1;
const int MS = 1;

int main(int argc, char* argv[])
{

    int i;

    // setup encoders:
    fd = openGPMCFPGA();
    printf("fd = %d\n", fd);
    if (0 > fd)
    {
        printf("Error: Could not open GPMC device.\n");
        return 1;
    }
    
    for (i = 0; i < 2; ++i)
    {
        ramstixInitDAC(fd, i);

        printf("DAC %i initialized and reset.\n", i);
    }
    while (run_test) {
        for (i = 0; i < 2; i++) {
            ramstixSetDACValue(fd, i, 0);
        }
    }
    
    
    for (i = 0; i < 2; ++i)
    {
        ramstixCloseQCounter(fd, i);
        ramstixCloseDAC(fd, i);
        printf("Encoder %i uninitialized.\n", i);
    }
    closeGPMCFPGA(fd);
    return 0;
}
