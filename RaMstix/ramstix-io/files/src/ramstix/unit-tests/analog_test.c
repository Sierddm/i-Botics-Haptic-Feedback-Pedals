/**
 * @file analog_test.c
 * @brief Test tool for the analog input and output devices.
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include <stdio.h>
#include <time.h>
#include <limits.h>

#include "ramstix/ramstix_analog.h"

int run_test;
int fd;
const int NUM_SECONDS = 1;
const int MS = 1;



int main(int argc, char* argv[])
{

    
    int i;
    
    printf("\nAnalog test\nPrint analog input and alter analog output every second.\n\n");
   
    run_test = 1;
    fd = openGPMCFPGA();
	if (0 > fd)
	{
		printf("Error: Could not open GPMC device.\n");
		return 1;
	}
	for (i = 0; i < 2; ++i)
    {
        ramstixInitADC(fd, i);

        printf("ADC %i initialized and.\n", i);
    }
    
    int count = 1;
    double time_counter = 0;
    double checkV;

    clock_t this_time = clock();
    clock_t last_time = this_time;

    printf("Gran = %ld\n", NUM_SECONDS * (CLOCKS_PER_SEC/MS));
    printf("ADC MAX VALUE: %lf, SHRT_MAX: %d\n", ADC_MAX_VALUE, SHRT_MAX);
    
    
    while (run_test)
    {
        this_time = clock();
        time_counter += (double)(this_time - last_time);
        last_time = this_time;

        if(time_counter > (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS)))
        {
            time_counter -= (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS));
            float time_passed = count;
            printf("== Performing test @ %.3fs:\n", time_passed);

            for (i = 0; i < 2; ++i)
            {
                checkV = ( (ADC_MAX_VALUE / SHRT_MAX) * ramstixGetADCIntValue(fd, i));
            	printf("Analog in %i: %f V, int: %i, calcV: %lf\n", i+1, ramstixGetADCValue(fd, i), ramstixGetADCIntValue(fd, i), checkV);
            }   
            count++;
        }
    }
	
    closeGPMCFPGA(fd);
    return 0;
}
