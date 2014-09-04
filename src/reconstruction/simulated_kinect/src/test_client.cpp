#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_msgs/MoveCmd.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
 
  ros::NodeHandle n;
  ros::Rate r(10);
   ros::Publisher _pub = n.advertise<robot_msgs::MoveCmd>("/ROS_Sensor/moveCmd", 1000);
   robot_msgs::MoveCmd msg;
   msg.acc = 0.1; msg.vel = 1.2;
   msg.cmdId = 200;
   msg.moveType = 0;
   
   msg.joint.q.push_back(2.00);
   msg.joint.q.push_back(-1.571);
   msg.joint.q.push_back(-1.571);
   msg.joint.q.push_back(-1.571);
   msg.joint.q.push_back(1.571);
   msg.joint.q.push_back(1.571);
   
   for(int i= 0;i <= 5;i++){
   _pub.publish(msg);
   r.sleep();
   ROS_INFO("publishing ..... %d", i);
   }
   
  return 0;
}