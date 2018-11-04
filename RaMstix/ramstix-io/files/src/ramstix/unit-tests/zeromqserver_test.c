#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include "ramstix/ramstix_qcntr.h"
#include "ramstix/ramstix_analog.h"
#include "ramstix/zhelpers.h"

const double    offsetAngle_left    = 22.76;
const double    offsetAngle_right   = 21.82;
const double    max_step_left       = 16653;
const double    max_step_right      = 16314;
const double    max_angle_left      = 53.29;
const double    max_angle_right     = 52.02;
const double    pedalRadius         = 0.115;
const double    degToRad            = 0.0174532925;
const double    gravity             = 9.81;
const double    intToVol            = 0.000153023;

int main (int argc, char* argv[])
{
    //  Socket to talk to clients
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_REP);
    int rc = zmq_bind (responder, "tcp://*:5555");
    assert (rc == 0);
    
    int i;
    int fd;
    double stepToDeg_left = (max_angle_left - offsetAngle_left) / max_step_left;
    double stepToDeg_right = (max_angle_right - offsetAngle_right) / max_step_right;
    double stepToRad_left = stepToDeg_left * degToRad;
    double stepToRad_right = stepToDeg_right * degToRad;
    double pedalOffset_left = offsetAngle_left * degToRad;
    double pedalOffset_right = offsetAngle_right * degToRad;

    // setup FPGA connection:
    fd = openGPMCFPGA();
    printf("fd = %d\n", fd);
    if (0 > fd)
    {
        printf("Error: Could not open GPMC device.\n");
        return 1;
    }
    
    // Initiate encoders, DACs and ADCs
    for (i = 0; i < 2; ++i)
    {
        ramstixInitDAC(fd, i);
        printf("DAC %i initialized and reset.\n", i);
        ramstixInitADC(fd, i);
        printf("ADC %i initialized and reset.\n", i);
        ramstixInitQCounter(fd, i, NO_RESET, QUADRATURE);
        ramstixSetQCounterValue(fd, i, 0);
        printf("ENC %i initialized and reset.\n", i);
    }

    while (1) {
        // Receiving DAC values
        char *string = s_recv (responder);
        double dacL, dacR;
        sscanf (string, "%lf %lf", &dacL, &dacR);
        free (string);
/*        printf ("R:VOL L%.3lf\tR%.3lf\n", dacL, dacR);*/
        ramstixSetDACValue(fd, 1, dacL);
        ramstixSetDACValue(fd, 0, dacR);
        
        // Reading ADC and encoder values from registers
        int encL, encR;
        int accL, accR;
        encL = -ramstixGetQCounterValue(fd, 1);
        encR = ramstixGetQCounterValue(fd, 0);
        accL = ramstixGetADCIntValue(fd, 1);
        accR = ramstixGetADCIntValue(fd, 0);
        
        double volL = accL * intToVol;
        double volR = accR * intToVol;
        double angL = (double)encL * stepToDeg_left + offsetAngle_left;
        double angR = (double)encR * stepToDeg_right + offsetAngle_right;
    
        //  Responding with ADC and encoder values
        char update [50];
        sprintf (update, "%d %d %lf %lf", encL, encR, volL, volR);
/*        printf("INT: %d\t%d\n", accL, accR);*/
        printf("S:VOL L%.3lf\tR%.3lf\n", volL, volR);
/*        printf("S:ACC L%.5d\tR%.5d\n", accL, accR);*/
/*        printf("S:ENC L%.5d\tR%.5d\n", encL, encR);*/
/*        printf("S:ANG L%.2lf\tR%.2  lf\n", angL, angR);*/
        s_send (responder, update);
    }
    
    zmq_close (responder);
    zmq_ctx_destroy (context);
    for (i = 0; i < 2; ++i)
    {
        ramstixCloseQCounter(fd, i);
        ramstixCloseDAC(fd, i);
        ramstixCloseADC(fd, i);
        printf("Encoder, ADC and DAC %i uninitialized.\n", i);
    }
    closeGPMCFPGA(fd);
    return 0;
}
