#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "haptic_pedals/Forces.h"
#include "haptic_pedals/EncoderValues.h"
#include "haptic_pedals/RequestForce.h"

#include <stdio.h>
#include <math.h>

// === USER CHANGEABLE VARIABLES ===
const int       programFrequecy     = 100;
const int       filterArrayLength   = 3;
const int       offsetAngle         = 17;
const int       max_step            = 17000;
const double    voltage_offsetL     = 1.65;
const double    voltage_offsetR     = 1.65;
const double    max_angle           = 38.57;
const double    pedalRadius         = 0.115;
const double    volPerG             = 0.3;
// =================================

const int       encL_offset = 0;
const int       encR_offset = 1;
const int       volL_offset = 0;
const int       volR_offset = 1;
const double    degToRad    = 0.0174532925;
const double    gravity     = 9.81;
int             count       = 0;

void hardwareInitialize();
void hardwareRequest();
void convertHardware();
void getVelocity();
void requestForce(ros::ServiceClient);
void forceFeedback(double, double);
void publishEncoders(ros::Publisher);
double differentiate(double[], double);
double integrate(double[], double);
double lowPassFilter(double[], double[]);
double highPassFilter(double);


int i, fd;
double forceL, forceR, pedalOffset, stepToRad;
int encL [filterArrayLength], encR [filterArrayLength];
double volL [filterArrayLength], volR [filterArrayLength];
double posL [filterArrayLength], posR [filterArrayLength];
double accL [filterArrayLength], accR [filterArrayLength];
double velL_enc [filterArrayLength], velR_enc [filterArrayLength];
double velL_acc [filterArrayLength], velR_acc [filterArrayLength];
double velL_enc_f [filterArrayLength], velR_enc_f [filterArrayLength];
double velL_acc_f [filterArrayLength], velR_acc_f [filterArrayLength];
double velL [filterArrayLength], velR [filterArrayLength];
double cosL, cosR;
double stepToDeg;

#define NZEROS 2
#define NPOLES 2
#define GAIN_LP   4.840925170e+00
#define GAIN_HP   2.555350342e+00




  /*    TODO
    - Initialize GPMCFPGA
    - Initialize encoders
    - Initialize ADC's 
    - Request encoder data
    - Request ADC data
    - Create filters
    - Process force from MSD
    - Provide force feedback to pedals
  */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "haptic_pedals_driver");
    ros::NodeHandle n;
    ros::Publisher encoder_pub = n.advertise<haptic_pedals::EncoderValues>("EncoderValues", 1000);
    ros::ServiceClient movement_client = n.serviceClient<haptic_pedals::RequestForce>("RequestForce");
    ros::Rate loop_rate(programFrequecy);

    hardwareInitialize();   
  
    while (ros::ok())
    { 
        hardwareRequest();
        publishEncoders(encoder_pub);
        requestForce(movement_client);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


  return 0;
}

void hardwareInitialize() {

    // Setup required variables based on set variables
    stepToDeg = (max_angle - offsetAngle) / max_step;
    stepToRad = stepToDeg * degToRad;
    pedalOffset = offsetAngle * degToRad;
    for (i = 0; i < filterArrayLength; i++) {
        encL[i] = 0;
        encR[i] = 0;
        volL[i] = 0;
        volR[i] = 0;
        posL[i] = 0;
        posR[i] = 0;
        accL[i] = 0;
        accR[i] = 0;
        velL[i] = 0;
        velR[i] = 0;
        velL_acc[i] = 0;
        velR_acc[i] = 0;
        velL_enc[i] = 0;
        velR_enc[i] = 0;
    }
    ROS_INFO("Set all arrays to 0");
}

void hardwareRequest(){
    // move all array values one place up 
    for (i = filterArrayLength- 1; i > 0; i--) {
        encL[i] = encL[i-1];
        encR[i] = encR[i-1];
        volL[i] = volL[i-1];
        volR[i] = volR[i-1];
    }
    
    // acquire new hardware input
//    encL[0] = ramstixGetQCounterValue(fd, encL_offset);
//    encR[0] = ramstixGetQCounterValue(fd, encR_offset);
//    volL[0] = ramstixGetADCIntValue(fd, volL_offset);
//    volR[0] = ramstixGetADCIntValue(fd, volR_offset);

    encL[0] = 6000;
    encR[0] = 10000;
    volL[0] = 2;
    volR[0] = 2;

    convertHardware();
}

void convertHardware(){
    for (i = 0; i < filterArrayLength; i++) {
        // convert encoder output to radians and include the pedal starting position offset
        posL[i] = encL[i] * stepToRad + pedalOffset;
        posR[i] = encR[i] * stepToRad + pedalOffset;
        
        // center voltage around offset, compensate for gravity and devide by pedal radius for acceleration
        accL[i] = ((volL[i] - voltage_offsetL) / volPerG * gravity - (cos(posL[i]) * gravity)) / pedalRadius;
        accR[i] = ((volR[i] - voltage_offsetR) / volPerG * gravity - (cos(posR[i]) * gravity)) / pedalRadius;
    }

    getVelocity();
}

void getVelocity(){
    // move all array values one place up 
    for (i = filterArrayLength- 1; i > 0; i--) {
        velL_enc[i] = velL_enc[i-1];
        velR_enc[i] = velR_enc[i-1];
        velL_acc[i] = velL_acc[i-1];
        velR_acc[i] = velR_acc[i-1];
    }
    
    // calculate new velocity values with differentiate and integrate
    velL_enc[0] = differentiate(posL, velL_enc[1]);
    velR_enc[0] = differentiate(posR, velR_enc[1]);
    
    double temp[filterArrayLength];
    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velL_enc_f[i];
    }
    velL_enc_f[2] = velL_enc_f[1]; velL_enc_f[1] = velL_enc_f[0];
    velL_enc_f[0] = lowPassFilter(velL_enc, temp);
    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velR_enc_f[i];
    }
    velR_enc_f[2] = velR_enc_f[1]; velR_enc_f[1] = velR_enc_f[0];
    velR_enc_f[0] = lowPassFilter(velR_enc, temp);
    
//    ROS_INFO("======== integration/differentiation: \n");
//    ROS_INFO("diff: ");
//    for (i = 0; i < filterArrayLength; i++){
//        ROS_INFO("[%.4lf] ", velL_enc[i]);
//    }
//    ROS_INFO("\n");
//    ROS_INFO("int:  ");
//    for (i = 0; i < filterArrayLength; i++){
//        ROS_INFO("[%.4lf] ", velL_acc[i]);
//    }
//    ROS_INFO("\n");
}

double differentiate(double pos[], double vel){
    return vel + ((pos[0] - pos[1]) / (1/(double)programFrequecy));
}

double integrate(double acc[], double vel){
    return ((acc[0] + acc[1]) / 2) * (1/(double)programFrequecy);
}

void requestForce(ros::ServiceClient client){
    haptic_pedals::RequestForce msg;
    msg.request.position_left = posL[0];
    msg.request.velocity_left = velL[0];
    msg.request.acceleration_left = accL[0];
    msg.request.position_right = posR[0];
    msg.request.velocity_right = velR[0];
    msg.request.acceleration_right = accR[0];
    
    if (client.call(msg)) {
        ROS_INFO("force L: %lf", msg.response.force_left);
        ROS_INFO("force R: %lf", msg.response.force_right);
        forceFeedback(msg.response.force_left, msg.response.force_right);
    }
    else {
        ROS_ERROR("Failed to call service RequestForce");
    }
}

void forceFeedback(double left, double right){
    // set DAC output to either left or right force 
}

void publishEncoders(ros::Publisher pub){
    haptic_pedals::EncoderValues msg;
    msg.encoder_left = (double)encL[0] / (double)max_step * 100 - 50;
    msg.encoder_right = (double)encR[0] / (double)max_step * 100 - 50;
    ROS_INFO("encoder(percentage) left:  %lf", msg.encoder_left);
    ROS_INFO("encoder(percentage) right: %lf", msg.encoder_right);
    pub.publish(msg);
}

double lowPassFilter(double xv[], double yv[]){
    for (;;){ 
        for (i = 0; i < filterArrayLength; i++) {
            xv[i] = xv[i] / GAIN_LP;
        }
        yv[2] = yv[1]; yv[1] = yv[0];
        yv[0] = (xv[2] + xv[0]) + 2 * xv[1]
                     + ( -0.1958157127 * yv[2]) + (  0.3695273774 * yv[1]);
        return yv[0];
      }
}

double highPassFilter(double xv[], double yv[]){
    for (;;)
      { 
      for (i = 0; i < filterArrayLength; i++) {
            xv[i] = xv[i] / GAIN_HP;
        }
        yv[2] = yv[1]; yv[1] = yv[0];
        yv[0] =   (xv[2] + xv[0]) - 2 * xv[1]
                     + ( -0.1958157127 * yv[2]) + (  0.3695273774 * yv[1]);
        return yv[0];
      }
}
