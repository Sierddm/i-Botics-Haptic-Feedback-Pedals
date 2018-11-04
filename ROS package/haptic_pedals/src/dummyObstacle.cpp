//
//  Simple node for faking obstacle messages
//
//
//
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummyObstacle");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("ObstacleLocation", 1000);

  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 6;
    msg.angular.z = 1;

    // Irrelevant
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

    ROS_INFO("Twist");
    ROS_INFO("[%f] [%f] [%f]", msg.linear.x, msg.linear.y, msg.linear.z);
    ROS_INFO("[%f] [%f] [%f]", msg.angular.x, msg.angular.y, msg.angular.z);

//    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
