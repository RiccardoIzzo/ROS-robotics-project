#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"

// Subscriber class
class Subscriber {
public:
  Subscriber(); 
  void main_loop();
  void countCallback(const sensor_msgs::JointState::ConstPtr& msg);
  geometry_msgs::Vector3 calcForwardVelocity(double x, double y);
  geometry_msgs::Vector3 calcAngularVelocity(double z);

private:
  ros::NodeHandle n; 
  ros::Subscriber sub;
  ros::Publisher pub;
  geometry_msgs::Vector3 lin_vel;
  geometry_msgs::Vector3 ang_vel;
  long int number_of_messages;
  double previous_positions[4];
  double previous_time_sec;
  double previous_time_nsec;
};

#endif