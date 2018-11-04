//
//  Node for converting the pedal displacement to a Twist
//  Used for controlling the Gazebo Turtlebot or the actual platform
//
//
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "haptic_pedals/EncoderValues.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

const double max_linear_speed = .5; //3.6
const double max_angular_speed = 2.0;
bool a = true;

ros::Publisher direction_pub;

void publishDirection(geometry_msgs::Twist);

void convertDirection(const haptic_pedals::EncoderValues::ConstPtr& msg){

    // ROS_INFO("received left:  %lf", msg->encoder_left);
    // ROS_INFO("received right: %lf", msg->encoder_right);

    double delta = msg->encoder_left - msg->encoder_right;
    double sum = msg->encoder_left + msg->encoder_right;

    // ROS_INFO("Sum:   %lf", sum);
    // ROS_INFO("Delta: %lf", delta);

    geometry_msgs::Twist twist;
    twist.angular.z = -(max_angular_speed / 100 * delta);
    twist.linear.x = max_linear_speed / 100 * sum;

    publishDirection(twist);
}

main(int argc, char **argv)
{
    ros::init(argc, argv, "pedal_Direction");
    ros::NodeHandle n;
    direction_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 0); //pedalDirection
    ros::Subscriber obstacle_sub = n.subscribe<haptic_pedals::EncoderValues>("EncoderValues", 0, convertDirection);

    ros::spin();

    return 0;
}

void publishDirection(geometry_msgs::Twist msg){
    direction_pub.publish(msg);
    if (a == true) {
        printf("linear x, angular z\n");
        a = false;
    }
    printf("%lf,", msg.linear.x);
    printf("%lf\n", msg.angular.z);
}
