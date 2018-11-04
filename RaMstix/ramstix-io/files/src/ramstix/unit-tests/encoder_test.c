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
        ramstixInitQCounter(fd, i, NO_RESET, QUADRATURE);
        ramstixSetQCounterValue(fd, i, 0);
        ramstixInitADC(fd, i);
        printf("Encoder %i initialized and reset.\n", i);
    }
    
/*    unitTest();*/
/*    int count = 1;*/

/*    double time_counter = 0;*/

/*    clock_t this_time = clock();*/
/*    clock_t last_time = this_time;*/

/*    printf("Gran = %ld\n", NUM_SECONDS * (CLOCKS_PER_SEC/MS));*/
/*    */
/*    int x = 0;*/
/*    */
    while (run_test) {
/*    */
/*        if (x == 65000){*/
/*            run_test = 0;*/
/*        }*/
/*        this_time = clock();*/
/*        time_counter += (double)(this_time - last_time);*/
/*        last_time = this_time;*/

/*        if(time_counter > (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS)))*/
/*        {*/
/*            time_counter -= (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS));*/
/*            float time_passed = count;*/
/*            printf("== Performing test @ %.3fs:\n", time_passed);*/
/*            printf("V = %d:\n", x);*/
/*            ramstixSetDACIntValue(fd, 1, 65535);*/
/*            for (i = 0; i < ENCODER_TOTAL_NUM; ++i)*/
/*            {*/
/*                printf("Encoder %i: %i\n", i+1, ramstixGetQCounterValue(fd, i));*/
/*            }*/
/*            count++;*/
/*            x+=2500;*/
/*        }*/
/*    */
/*        for (i= 0; i < 2; i++) {*/
            printf("%d\t%d\t%d\t%d\n", ramstixGetQCounterValue(fd, 0), ramstixGetQCounterValue(fd, 1), ramstixGetADCIntValue(fd, 0), ramstixGetADCIntValue(fd, 1));   
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
