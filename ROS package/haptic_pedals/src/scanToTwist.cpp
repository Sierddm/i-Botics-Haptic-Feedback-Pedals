//
//  Node for converting the Gazebo Turtlebot laserscanner values to vectors in a Twist message
//  This Node does not have to be used if the actual robot publishes obstacle values in Twists
//
//
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <math.h>

ros::Publisher twist_pub;
void publishTwist(geometry_msgs::Twist);

int i;
double max_range = 3.5;
int direction;

void storeDirection(const geometry_msgs::Twist::ConstPtr& msg){
    if (msg->linear.x > 0) {
        direction = 2;
    } else if (msg->linear.x < 0) {
        direction = 1;
    } else {
        direction = 0;
    }
}

void convertScan(const sensor_msgs::LaserScan::ConstPtr& msg){

    // Array length = 360, angle of 90 degree in driving direction
    geometry_msgs::Twist twist;
    double closest = max_range + .5;
    int angle = -1;
    if (direction == 2) {
        for (i = 0; i < 46; i++){
            if (msg->ranges[i] < closest) {
                closest = msg->ranges[i];
                angle = i;
            }
        }
        for (i = 315; i < 360; i++) {
            if (msg->ranges[i] < closest) {
                closest = msg->ranges[i];
                angle = i;
            }
        }
    } else if (direction == 1) {
        for (i = 135; i < 226; i++){
            if (msg->ranges[i] < closest) {
                closest = msg->ranges[i];
                angle = i;
            }
        }
    } else {
        for (i = 0; i < 360; i++){
            if (msg->ranges[i] < closest) {
                closest = msg->ranges[i];
                angle = i;
            }
        }
    }
    if (closest == max_range+.5){
        twist.angular.z = 0;
        twist.linear.x = 0;
    } else {
        double radians = angle * M_PI / 180;

//        if (angle >= 0 && angle < 90) {
//            twist.angular.z = -(cos(radians) * closest);
//            twist.linear.x =    sin(radians) * closest;
//        } else if (angle >= 90 && angle < 180) {
//            twist.angular.z =   cos(radians) * closest;
//            twist.linear.x =  -(sin(radians) * closest);
//        } else if (angle >= 180 && angle < 270) {
//            twist.angular.z =   cos(radians) * closest;
//            twist.linear.x =  -(sin(radians) * closest);
//        } else if (angle >= 270 && angle < 360) {
//            twist.angular.z =   cos(radians) * closest;
//            twist.linear.x =    sin(radians) * closest;
//        }

        twist.angular.z = -(sin(radians) * closest);
        twist.linear.x =    cos(radians) * closest;
    }
    printf("------------\n");
    printf("D: %d\n", direction);
    printf("A: %d\n", angle);
    printf("R: %.2lf\n", closest);
    printf("X: %.2lf\n", twist.linear.x);
    printf("Z: %.2lf\n", twist.angular.z);

    publishTwist(twist);

}

main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_twist");
    ros::NodeHandle n;
    twist_pub = n.advertise<geometry_msgs::Twist>("ObstacleLocation", 1000);
    ros::Subscriber direction_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, storeDirection);
    ros::Subscriber obstacle_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, convertScan);

    ros::spin();

    return 0;
}

void publishTwist(geometry_msgs::Twist msg){
    twist_pub.publish(msg);
}
