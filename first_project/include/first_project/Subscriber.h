#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "first_project/Reset.h"

// Subscriber class
class Subscriber {
enum IntegrationMethod {Euler, RK}; 

public:
  Subscriber(); 
  void main_loop();
  void countCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void controlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  geometry_msgs::Vector3 calcForwardVelocity(double x, double y);
  geometry_msgs::Vector3 calcAngularVelocity(double z);
  geometry_msgs::Vector3 computeEuler(double delta_t);
  geometry_msgs::Vector3 computeRungeKutta(double delta_t);

  // global parameters: must be visible to both the launch file and the service callback function in Odometry.cpp
  double x_k;
  double y_k;
  double theta;

  ros::NodeHandle n;

private:
  
  ros::Subscriber sub;
  ros::Publisher pub;
  geometry_msgs::Vector3 lin_vel;
  geometry_msgs::Vector3 ang_vel;
  geometry_msgs::Vector3 parameters;
  long int number_of_messages;
  double previous_positions[4];
  double v_x;
  double v_y;
  double w_z;
  double previous_time_sec;
  double previous_time_nsec;
  double lastTime;
  IntegrationMethod integMethod;
};

#endif
