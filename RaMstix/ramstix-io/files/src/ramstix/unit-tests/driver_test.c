/**
 * @file driver_analog.c
 * @brief
 * @author Sierd Meijer, University of Twente
 */
 
#include "ramstix/ramstix_analog.h"
#include "ramstix/ramstix_qcntr.h"
#include "ramstix/ramstix_definitions.h"
#include <limits.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

void initHardware();
double * getHardwareVal();
void closeHardware();

int i;
int fd;
int run_test = 1;
const int NUM_SECONDS = 50;
const int MS = 1000;
double stepToDeg;
double degToRad;
double stepToRad;
double radOffset;

struct sigaction sigIntHandler;

int main(int argc, char* argv[])
{
/*    initHardware();*/
/*    printf("MILESTONE2");*/
/*    while (run_test) */
/*    {*/
/*        for (i = 0; i < 4; ++i) */
/*        {*/
/*            printf("%lf", getHardwareVal()[i]);*/
/*        }*/
/*    }*/

    fd = openGPMCFPGA();
	if (0 > fd)
	{
		printf("Error: Could not open GPMC device.\n");
	}
	for (i = 0; i < 2; ++i)
    {
        ramstixInitADC(fd, i);
        printf("ADC %i initialized.\n", i);
    }
    for (i = 0; i < 2; ++i)
    {
        ramstixInitQCounter(fd, i, NO_RESET, QUADRATURE);
        ramstixSetQCounterValue(fd, i, 0);
        printf("Encoder %i initialized and reset.\n", i);
    }

    static double values[4];
    int count = 1;
    double time_counter = 0;
    stepToDeg = .001256627;
    degToRad = 0.0174532925;
    stepToRad = stepToDeg * degToRad;
    radOffset = 17 * degToRad;

    clock_t this_time = clock();
    clock_t last_time = this_time;
    
    while (run_test)
    {
        this_time = clock();
        time_counter += (double)(this_time - last_time);
        last_time = this_time;
        

        if(time_counter > (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS)))
        {
            time_counter -= (double)(NUM_SECONDS * (CLOCKS_PER_SEC/MS));
            float time_passed = count * NUM_SECONDS / MS;
/*            printf("== Performing test @ %.2fs:\n", time_passed);*/
            for (i = 0; i < 2; ++i)
            {
/*                printf("enc %d:\t %5d | adc %d:\t %5d\n", i, ramstixGetQCounterValue(fd, i), i, ramstixGetADCIntValue(fd, i));*/
                values[i] = (double)ramstixGetQCounterValue(fd, i);
                values[i+2] = (double)ramstixGetADCIntValue(fd, i);
            }
            for (i = 1; i < 2; ++i)
            {
                double deg = values[i] * stepToDeg + 17;
                double volt = ( (ADC_MAX_VALUE / SHRT_MAX) * values[i+2] );
/*                printf("enc %d: %5lf | adc %d: %5lf\n", i, deg, i, volt);*/
                printf("%lf\n", volt);
            }
            fflush(stdout);
            count++;
        }
    }

    return 0;
}

double * getHardwareVal()
{
    static double values[4];
    for (i = 0; i < ENCODER_TOTAL_NUM; ++i)
    {
        values[i] = ramstixGetQCounterValue(fd, i);
        values[i+2] = ramstixGetADCIntValue(fd, i);
    }
    for (i = 0; i < 4; ++i) 
    {
        printf("%lf", values[i]);
    }
    return values;
}
    
    
// Initializes the connected hardware
void initHardware() 
{
    fd = openGPMCFPGA();
	if (0 > fd)
	{
		printf("Error: Could not open GPMC device.\n");
	}
	for (i = 0; i < 2; ++i)
    {
        ramstixInitADC(fd, i);
        printf("ADC %i initialized.\n", i);
    }
    for (i = 0; i < ENCODER_TOTAL_NUM; ++i)
    {
        ramstixInitQCounter(fd, i, NO_RESET, QUADRATURE);
        ramstixSetQCounterValue(fd, i, 0);
        printf("Encoder %i initialized and reset.\n", i);
    }
    
    printf("MILESTONE");
    // Catches cntl+c and calls closeHardware()
//    sigIntHandler.sa_handler = closeHardware;*/
//    sigemptyset(&sigIntHandler.sa_mask);*/
//    sigIntHandler.sa_flags = 0;*/
//    sigaction(SIGINT, &sigIntHandler, NULL);*/
    printf("MILESTONE");
}

// Closes all initialized hardware and exists
void closeHardware(int s)
{
    printf("Caught signal %d\n", s);
    closeGPMCFPGA(fd);
    for (i = 0; i < 2; ++i)
    {
        ramstixCloseADC(fd, i);

        printf("ADC %i closed.\n", i);
    }
    for (i = 0; i < ENCODER_TOTAL_NUM; ++i)
    {
        ramstixCloseQCounter(fd, i);

        printf("Encoder %i closed.\n", i);
    }
    exit(1);
}
