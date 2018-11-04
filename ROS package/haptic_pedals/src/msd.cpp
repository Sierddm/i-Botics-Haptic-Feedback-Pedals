//
//  The node for calculating the virtual Mass Spring Damper
//  Here the damping ratio can be tuned
//
//
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "haptic_pedals/Forces.h"
#include "haptic_pedals/Movement.h"
#include "haptic_pedals/RequestForce.h"

#include <stdio.h>
#include <math.h>

double massSpringDamper(double, double, double, double, double);

double posL, velL, accL, posR, velR, accR;
double forceL, forceR, distance, max_distance, platform_velocity;
double max_speed = 3.6;
double mass, spring, damper;
double stiffness_increase, stiffness_increase_left, stiffness_increase_right, damper_inc_left, damper_inc_right;
double origin_pos_left = ((53.29 - 22.76) / 2 + 22.76) * 0.0174532925; // = 0,663661447
double origin_pos_right = ((52.02 - 21.82) / 2 + 21.82) * 0.0174532925;// = 0,644375559
double max = 0;
double inertia = 0.5333333333333333;
double spring_constant = 6;
double damper_constant = .29; //.29
bool a = true;

// Receiving the position, velocity and acceleration and returning the output force for both pedals
bool returnForce(haptic_pedals::RequestForce::Request &req, haptic_pedals::RequestForce::Response &res)
{

    posL = req.position_left;
    velL = req.velocity_left;
    accL = req.acceleration_left;
    posR = req.position_right;
    velR = req.velocity_right;
    accR = req.acceleration_right;

    res.force_left = massSpringDamper(posL, velL, damper_inc_left, stiffness_increase_left, origin_pos_left);
    res.force_right = massSpringDamper(posR, velR, damper_inc_right, stiffness_increase_right, origin_pos_right);

    return true;
}

// Storing the feedback forces for both pedals upon receival and calculating the increased spring and damper output
void storeFeedbackForce(const haptic_pedals::Forces::ConstPtr& msg){
    // if (msg->distance > 0){
        forceL = msg->distribution_left;
        forceR = msg->distribution_right;
        platform_velocity = msg->platform_velocity + 0.5;
        distance = msg->distance;
        max_distance = 3.5;
        if (distance < max_distance && distance > 0) {
            stiffness_increase = 6 / (distance + .4) - 1.55;
            // stiffness_increase = 10;
        } else {
            stiffness_increase = 0;
        }
        stiffness_increase_left  = stiffness_increase * forceL * platform_velocity + spring_constant;
        stiffness_increase_right = stiffness_increase * forceR * platform_velocity + spring_constant;
        damper_inc_left  = damper_constant * sqrt(inertia * stiffness_increase_left);
        damper_inc_right = damper_constant * sqrt(inertia * stiffness_increase_right);
    // }

    double damping_ratio = damper_inc_right/(2 * sqrt(inertia * stiffness_increase_right));

//    ROS_INFO("increase: %lf", stiffness_increase_right);
    // ROS_INFO("OUTPUT forces =======");
    // ROS_INFO("Distance:    [%.3lf]", msg->distance);
    // ROS_INFO("Pvelocity:   [%.3lf]", msg->platform_velocity);
    // ROS_INFO("stiff_inc:   [%.3lf]", stiffness_increase);
    // ROS_INFO("stiff_inc_r: [%.3lf]", stiffness_increase_right);
    // ROS_INFO("damp_inc_r:  [%.3lf]", damper_inc_right);
    // ROS_INFO("DampingR:    [%.3lf]", damping_ratio);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msd");
    ros::NodeHandle n;
    ros::Subscriber forces_sub = n.subscribe("Forces", 0, storeFeedbackForce);
    ros::ServiceServer movement_service = n.advertiseService("RequestForce", returnForce);

    ros::spin();

    return 0;
}

double massSpringDamper(double pos, double vel, double damper, double spring, double origin_pos){

    // function for spring calculation with feedback

    double displacement = pos - origin_pos;
    double voltage_offset = .5;

    // ROS_INFO("DisplacementA [%.3lf]", displacement);

    if (displacement >= 0) {
       displacement = .5*sqrt(displacement);
        // displacement = displacement;
    } else {
       displacement = -.5*sqrt(-displacement);
        // displacement = displacement;
    }

    // ROS_INFO("DisplacementB [%.3lf]", displacement);

    double spring_force = spring * displacement;
    double damper_force = damper * vel;
    double force = spring_force + damper_force - voltage_offset;

    double abso = sqrt(pow(damper_force, 2));
    if (abso > max) {
        max = abso;
    }

    if (origin_pos == origin_pos_right) {
        ROS_INFO("stiffinc: [%.3lf]", stiffness_increase_right);
        ROS_INFO("spring:   [%.3lf]", spring_force);
        ROS_INFO("damper:   [%.3lf]", damper_force);
        ROS_INFO("force:    [%.3lf]", force);
        // ROS_INFO("max:      [%.3lf]", max);
    }


    return force;
}
