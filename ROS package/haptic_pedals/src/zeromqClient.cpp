//
//  Driver Node for Haptic Feedback Pedals using ZeroMQ
//  Connects to the RaMStix on IP: 10.0.5.28:5555
//
//
#include "ros/ros.h"
#include "zhelpers.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "haptic_pedals/Forces.h"
#include "haptic_pedals/EncoderValues.h"
#include "haptic_pedals/RequestForce.h"

#include <string>
#include <iostream>
#include <math.h>

// === USER CHANGEABLE VARIABLES ===
const int       programFrequecy     = 200;
const int       filterArrayLength   = 3;
const int       offsetAngle_left    = 22.76;
const int       offsetAngle_right   = 21.82;
const int       max_step_left       = 16653;
const int       max_step_right      = 16314;
const double    voltage_offsetL     = 1.65;
const double    voltage_offsetR     = 1.4195;
const double    max_angle_left      = 53.29;
const double    max_angle_right     = 52.02;
const double    pedalRadius         = 0.115;
const double    volPerG             = 0.639039359;
// =================================

const double    degToRad    = 0.0174532925;
const double    gravity     = 9.81;
int             count       = 0;
double          dacL        = 0;
double          dacR        = 0;

void variableReset();
void convertHardware();
void getVelocity();
void requestForce(ros::ServiceClient);
void forceFeedback(double, double);
void publishEncoders(ros::Publisher);
double differentiate(double[]);
double integrate(double[], double);
double lowPassFilter(double[], double[]);
double lowPassFilter2(double[], double[]);
double highPassFilter(double[], double[]);

int i, fd;
double timer = 0.0;
double forceL, forceR, pedalOffset_left, pedalOffset_right, stepToRad_left, stepToRad_right;
int encL [filterArrayLength], encR [filterArrayLength];
double volL [filterArrayLength], volR [filterArrayLength];
double posL [filterArrayLength], posR [filterArrayLength];
double accL [filterArrayLength], accR [filterArrayLength];
double velL_enc [filterArrayLength], velR_enc [filterArrayLength];
double velL_acc [filterArrayLength], velR_acc [filterArrayLength];
double velL_enc_f [filterArrayLength], velR_enc_f [filterArrayLength];
double velL_acc_f [filterArrayLength], velR_acc_f [filterArrayLength];
double velL [filterArrayLength], velR [filterArrayLength];
double xv[filterArrayLength], yv[filterArrayLength], xh[filterArrayLength], yh[filterArrayLength];
double velR_acc_test[filterArrayLength];
double cosL, cosR;
double stepToDeg_left, stepToDeg_right;
double max = 0;
double min = 10;
bool abc = true;


int main (int argc, char **argv)
{
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REQ);
    // ROS_INFO("Connecting to RaMstix socket");
    socket.connect ("tcp://10.0.5.28:5555");

    // Prepare publisher and service communication
    ros::init(argc, argv, "haptic_pedals_driver");
    ros::NodeHandle n;
    ros::Publisher encoder_pub = n.advertise<haptic_pedals::EncoderValues>("EncoderValues", 0);
    ros::ServiceClient movement_client = n.serviceClient<haptic_pedals::RequestForce>("RequestForce");
    ros::Rate loop_rate(programFrequecy);

    // Set all arrays to required values
    variableReset();

    //  Do 10 requests, waiting each time for a response
    while (ros::ok()) {
        // Sending force voltages
        zmq::message_t request (20);
        snprintf ((char *) request.data(), 20 , "%lf %lf", dacL, dacR);
        socket.send(request);
//        ROS_INFO("SENDING:  %lf %lf", dacL, dacR);

        // Putting all values one up in the array
        for (i = filterArrayLength- 1; i > 0; i--) {
            encL[i] = encL[i-1];
            encR[i] = encR[i-1];
            volL[i] = volL[i-1];
            volR[i] = volR[i-1];
        }

        //  Get the reply.
        zmq::message_t reply;
        socket.recv (&reply);
        std::istringstream iss(static_cast<char*>(reply.data()));
        iss >> encL[0] >> encR[0] >> volL[0] >> volR[0];
//        ROS_INFO("RECEIVED: %d %d %f %f", encL[0], encR[0], volL[0], volR[0]);

        // Never let encoder values go below 0
        if (encL[0] < 0) {
            encL[0] = 0;
        }
        if (encR[0] < 0) {
            encR[0] = 0;
        }

        convertHardware();
        publishEncoders(encoder_pub);
        requestForce(movement_client);

        // Debug prints
        // printf("%lf,", timer);
        // printf("%lf\n", volR[0]);
        // printf("%lf,", posR[0] / degToRad);
        // printf("%lf,", (volR[0] - voltage_offsetR) / volPerG * gravity);
        // printf("%lf\n", accR[0] * pedalRadius);
        timer += 1.0/programFrequecy;

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}

// Reset all variables to 0 at beginning of program
void variableReset(){
    stepToDeg_left = (max_angle_left - offsetAngle_left) / max_step_left;
    stepToDeg_right = (max_angle_right - offsetAngle_right) / max_step_right;
    stepToRad_left = stepToDeg_left * degToRad;
    stepToRad_right = stepToDeg_right * degToRad;
    pedalOffset_left = offsetAngle_left * degToRad;
    pedalOffset_right = offsetAngle_right * degToRad;
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
        velL_acc_f[i] = 0;
        velR_acc_f[i] = 0;
        velL_enc_f[i] = 0;
        velR_enc_f[i] = 0;
        xv[i] = 0;
        yv[i] = 0;
        xh[i] = 0;
        yh[i] = 0;
    }
    // ROS_INFO("Set all arrays to 0");
}

// Convert measured hardware values to SI units
void convertHardware(){
    for (i = 0; i < filterArrayLength; i++) {
        // convert encoder output to radians and include the pedal starting position offset
        posL[i] = encL[i] * stepToRad_left + pedalOffset_left;
        posR[i] = encR[i] * stepToRad_right + pedalOffset_right;

        // center voltage around offset, compensate for gravity and devide by pedal radius for acceleration
        accL[i] = ((volL[i] - voltage_offsetL) / volPerG * gravity + (cos(posL[i]) * gravity)) / pedalRadius;
        accR[i] = ((volR[i] - voltage_offsetR) / volPerG * gravity + (cos(posR[i]) * gravity)) / pedalRadius;
    }

    // Debug prints
    printf("%lf,", timer);
    printf("%lf,", posR[0]);
    printf("%lf,", volR[0]);
    printf("%lf,", (volR[0] - voltage_offsetR) / volPerG * gravity / pedalRadius);
    printf("%lf,", accR[0]);
    printf("\n");

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

    // Integrate and differentiate to velocity
    velL_acc[0] = integrate(accL, velL_acc[0]);
    velR_acc[0] = integrate(accR, velR_acc[0]);
    velL_enc[0] = differentiate(posL);
    velR_enc[0] = differentiate(posR);

    // Filter differentiated and integrated values using previous filtered and unfiltered values
    double temp[filterArrayLength];
    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velL_enc_f[i];
    }
    // velL_enc_f[4] = velL_enc_f[3]; velL_enc_f[3] = velL_enc_f[2];
    velL_enc_f[2] = velL_enc_f[1]; velL_enc_f[1] = velL_enc_f[0];
    velL_enc_f[0] = lowPassFilter2(velL_enc, temp);
    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velR_enc_f[i];
    }
    // velR_enc_f[4] = velR_enc_f[3]; velR_enc_f[3] = velR_enc_f[2];
    velR_enc_f[2] = velR_enc_f[1]; velR_enc_f[1] = velR_enc_f[0];
    velR_enc_f[0] = lowPassFilter2(velR_enc, temp);

    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velL_acc_f[i];
    }
    // velL_acc_f[4] = velL_acc_f[3]; velL_acc_f[3] = velL_acc_f[2];
    velL_acc_f[2] = velL_acc_f[1]; velL_acc_f[1] = velL_acc_f[0];
    velL_acc_f[0] = highPassFilter(velL_acc, temp);
    for (i = 0; i < filterArrayLength; i++) {
        temp[i] = velR_acc_f[i];
    }
    // velR_acc_f[4] = velR_acc_f[3]; velR_acc_f[3] = velR_acc_f[2];
    velR_acc_f[2] = velR_acc_f[1]; velR_acc_f[1] = velR_acc_f[0];
    velR_acc_f[0] = highPassFilter(velR_acc, temp);

    velL[0] = velL_enc_f[0];
    velR[0] = velR_enc_f[0];

    // Debug prints
    // if (timer == 0) {
    //     printf("timer, angle, enc, ecn_f, acc, acc_f, fusion\n");
    // }
    // printf("%lf,", timer);
    // printf("%lf,", posR[0]);
    // printf("%lf,", velR_enc[0]);
    // printf("%lf,", velR_enc_f[0]);
    // printf("%lf,", accR[0]);
    // printf("%lf,", velR_acc[0]);
    // printf("%lf,", velR_acc_f[0]);
    // printf("%lf,", velR[0]);
    // printf("\n");
    // timer += 1;

}

// Differentiation
double differentiate(double pos[]){
    return (pos[0] - pos[1]) / (1/(double)programFrequecy);
}

// Integration
double integrate(double acc[], double vel){
    return vel + ((acc[0] + acc[1]) / 2) * (1/(double)programFrequecy);
}

// Publish the converted pedal position on EncoderValues
void publishEncoders(ros::Publisher pub){
    haptic_pedals::EncoderValues msg;
    msg.encoder_left  = -((double)encL[0] / (double)max_step_left * 100 - 50);
    msg.encoder_right = -((double)encR[0] / (double)max_step_right * 100 - 50);
    // printf("%lf,", msg.encoder_left);
    // printf("%lf\n", msg.encoder_right);
    pub.publish(msg);
}

// Request pedal forces based on send position, velocity and acceleration
void requestForce(ros::ServiceClient client){
    haptic_pedals::RequestForce msg;
    msg.request.position_left = posL[0];
    msg.request.velocity_left = velL[0];
    msg.request.acceleration_left = accL[0];
    msg.request.position_right = posR[0];
    msg.request.velocity_right = velR[0];
    msg.request.acceleration_right = accR[0];

    if (client.call(msg)) {
//        ROS_INFO("force L: %lf", msg.response.force_left);
//        ROS_INFO("force R: %lf", msg.response.force_right);
        forceFeedback(msg.response.force_left, msg.response.force_right);
    }
    else {
       ROS_ERROR("Failed to call service RequestForce");
    }
}

// Limit maximum voltage outputs
void forceFeedback(double left, double right){
    // set DAC output to either left or right force
    if (left > 5) {
        left = 5;
    }
    if (left < -5) {
        left = -5;
    }
    if (right > 5) {
        right = 5;
    }
    if (right < -5) {
        right = -5;
    }
    dacL = left;
    dacR = right;
}

// 200freq 10hz N=2
double lowPassFilter2(double xv[], double yv[]){
    double a[3] = {1, -1.56101807580072, 0.641351538057563};
    double b[3] = {0.0200833655642113, 0.0401667311284225, 0.0200833655642113};

    for (i = 0; i < filterArrayLength; i++) {
        xv[i] = xv[i] / a[0];
    }
    yv[2] = yv[1]; yv[1] = yv[0];
    yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2]
                    - a[1]*yv[1] - a[2]*yv[2];
    return yv[0];
}

// 200freq 10hz N=2
double highPassFilter(double xv[], double yv[]){
    double a[3] = {1, -1.56101807580072, 0.641351538057563};
    double b[3] = {0.800592403464570, -1.60118480692914, 0.800592403464570};

    for (i = 0; i < filterArrayLength; i++) {
        xv[i] = xv[i] / a[0];
    }
    yv[2] = yv[1]; yv[1] = yv[0];
    yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2]
                    - a[1]*yv[1] - a[2]*yv[2];
    return yv[0];
}

// // 200freq 30hz N=2
// double lowPassFilter2(double xv[], double yv[]){
//     double a[3] = {1, -.7478, 0.2722};
//     double b[3] = {0.1311, 0.2622, 0.1311};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2]
//                     - a[1]*yv[1] - a[2]*yv[2];
//     return yv[0];
// }

// // 200freq 30hz N=2
// double highPassFilter(double xv[], double yv[]){
//     double a[3] = {1, -.7478, 0.2722};
//     double b[3] = {0.5050, -1.0100, 0.5050};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2]
//                     - a[1]*yv[1] - a[2]*yv[2];
//     return yv[0];
// }

// 200freq 20hz N=4
// double lowPassFilter2(double xv[], double yv[]){
//     double a[filterArrayLength] = {1, -2.3695, 2.3140, -1.0547, 0.1874};
//     double b[filterArrayLength] = {0.0048, 0.0193, 0.0289, 0.0193, 0.0048};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[4] = yv[3]; yv[3] = yv[2]; yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2] + b[3]*xv[3] + b[4]*xv[4]
//                     - a[1]*yv[1] - a[2]*yv[2] - a[3]*yv[3] - a[4]*yv[4];
//     return yv[0];
// }

// // 200freq 20hz N=4
// double highPassFilter(double xv[], double yv[]){
//     double a[filterArrayLength] = {1, -2.3695, 2.3140, -1.0547, 0.1874};
//     double b[filterArrayLength] = {0.4328, -1.7314, 2.5971, -1.7314, 0.4328};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[4] = yv[3]; yv[3] = yv[2]; yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2] + b[3]*xv[3] + b[4]*xv[4]
//                     - a[1]*yv[1] - a[2]*yv[2] - a[3]*yv[3] - a[4]*yv[4];
//     return yv[0];
// }

// // 200freq 10hz N=4
// double lowPassFilter2(double xv[], double yv[]){
//     double a[filterArrayLength] = {1,-3.18063854887472,3.86119434899422,-2.11215535511097,0.438265142261981};
//     double b[filterArrayLength] = {0.000416599204406579,0.00166639681762631,0.00249959522643947,0.00166639681762631,0.000416599204406579};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[4] = yv[3]; yv[3] = yv[2]; yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2] + b[3]*xv[3] + b[4]*xv[4]
//                     - a[1]*yv[1] - a[2]*yv[2] - a[3]*yv[3] - a[4]*yv[4];
//     return yv[0];
// }

// // 200freq 10hz N=4
// double highPassFilter(double xv[], double yv[]){
//     double a[filterArrayLength] = {1,-3.18063854887472,3.86119434899422,-2.11215535511097,0.438265142261981};
//     double b[filterArrayLength] = {0.662015837202619,-2.64806334881047,3.97209502321571,-2.64806334881047,0.662015837202619};

//     for (i = 0; i < filterArrayLength; i++) {
//         xv[i] = xv[i] / a[0];
//     }
//     yv[4] = yv[3]; yv[3] = yv[2]; yv[2] = yv[1]; yv[1] = yv[0];
//     yv[0] = b[0]*xv[0] + b[1]*xv[1] + b[2]*xv[2] + b[3]*xv[3] + b[4]*xv[4]
//                     - a[1]*yv[1] - a[2]*yv[2] - a[3]*yv[3] - a[4]*yv[4];
//     return yv[0];
// }
