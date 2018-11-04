//
//  Node for calculating the increased force output based on the
//  distance and angle to an obstacle and the velocity of the platform
//
//
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "haptic_pedals/Forces.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"

#include <stdio.h>
#include <math.h>

const double max_speed = .5;
const double max_force = 100;
double velocity;
double velocity_magnitude;
bool inc_velocity = true;

ros::Publisher forces_pub;

void publishFeedback(haptic_pedals::Forces);
double forceDistribution(double);

void saveVelocity(const nav_msgs::Odometry::ConstPtr& msg){
    velocity = msg->twist.twist.linear.x;
}

void convertObstacle(const geometry_msgs::Twist::ConstPtr& msg){
    double magnitude = sqrt(pow(msg->linear.x, 2)+pow(msg->angular.z, 2));
    double locX = msg->linear.x / magnitude;
    double locY = msg->angular.z / magnitude;
    double angle = atan2(locY, locX) * 180 / M_PI;
    double conv = 0.555555556;
    double left, right;

    if (angle >= 0 && angle < 90) {
        // ROS_INFO("UP RIGHT -> %.2lf", angle);
        left = 50 - (angle * conv);
    } else if (angle >= 90 && angle <= 180) {
        // ROS_INFO("DOWN RIGHT -> %.2lf", angle);
        angle -= 90;
        left = (angle * conv);
    } else if (angle < 0 && angle >= -90) {
        // ROS_INFO("UP LEFT -> %.2lf", angle);
        angle = -angle;
        left = 50 + (angle * conv);
    } else if (angle < -90 && angle >= -180) {
        // ROS_INFO("DOWN LEFT -> %.2lf", angle);
        angle = -angle - 90;
        left = 100 - (angle * conv);
    }

    left = left / 100;
    right = 1 - left;

    // ROS_INFO("MAG: %.2lf", magnitude);

    if (inc_velocity){
        velocity_magnitude = .5 / max_speed * sqrt(pow(velocity, 2));
//        magnitude = magnitude * (.5 + velocity_magnitude);
    }

    // ROS_INFO("MAGB: %.2lf", magnitude);
    // ROS_INFO("LEFT: %.2lf - RIGHT: %.2lf", left, right);

    haptic_pedals::Forces forces;
    forces.distribution_left = left;
    forces.distribution_right = right;
    forces.distance = magnitude;
    forces.platform_velocity = velocity_magnitude;
    publishFeedback(forces);

}

main(int argc, char **argv)
{
    ros::init(argc, argv, "feedback_force");
    ros::NodeHandle n;
    forces_pub = n.advertise<haptic_pedals::Forces>("Forces", 0);
    ros::Subscriber obstacle_sub = n.subscribe<geometry_msgs::Twist>("ObstacleLocation", 0, convertObstacle);
    ros::Subscriber velocity_sub = n.subscribe<nav_msgs::Odometry>("odom", 0, saveVelocity);

    ros::spin();

    return 0;
}

void publishFeedback(haptic_pedals::Forces msg){
    forces_pub.publish(msg);
}
