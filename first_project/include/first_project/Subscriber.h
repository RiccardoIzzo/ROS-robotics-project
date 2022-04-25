#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

// Subscriber class
class Subscriber {
enum IntegrationMethod {Euler, RK}; 

public:
  Subscriber(); 
  void main_loop();
  void velocityCallback(const sensor_msgs::JointState::ConstPtr& msg);         //callback function used in Velocity.cpp
  void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);     //callback function used in Odometry.cpp
  void controlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);      //callback function used in Control.cpp
  geometry_msgs::Vector3 calcForwardVelocity(double x, double y);
  geometry_msgs::Vector3 calcAngularVelocity(double z);
  // Odometry functions to compute Euler and Runge-Kutta integration
  geometry_msgs::Vector3 computeEuler(double delta_t);
  geometry_msgs::Vector3 computeRungeKutta(double delta_t);

  // global parameters: must be visible to both the launch file and the service callback function in Odometry.cpp
  ros::NodeHandle n;
  // robot pose (x, y, ϑ)
  double x_k;
  double y_k;
  double theta;
  // integration method selector that can change with dynamic reconfigure
  int integMethod;

private:
  ros::Subscriber sub;
  ros::Publisher pub;
  long int number_of_messages;
  
  // Velocity.cpp variables
  geometry_msgs::Vector3 lin_vel;
  geometry_msgs::Vector3 ang_vel;
  geometry_msgs::Vector3 parameters;
  double previous_positions[4];
  double previous_time_sec;
  double previous_time_nsec;

  // Odometry.cpp variables
  double v_x;
  double v_y;
  double w_z;
  double lastTime;
};

#endif
