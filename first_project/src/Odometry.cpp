#include "first_project/Subscriber.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "math.h"

// Return a vector with three components
geometry_msgs::Vector3 asVector3(double x, double y, double z)
{
	auto v3 = geometry_msgs::Vector3();
	v3.x = x;
	v3.y = y;
	v3.z = z;

	return v3;
}

Subscriber::Subscriber() { // class constructor
  //subscriber that listen on "/cmd_vel" topic
  this->sub = this->n.subscribe("/cmd_vel", 1000, &Subscriber::odometryCallback, this);
  //publisher that publish a nav_msgs/Odometry message on "/odom" topic
  this->pub = this->n.advertise<nav_msgs::Odometry>("/odom", 1000);
  //by default the integration method is Euler
  this->integMethod = Euler;

  auto parameters = geometry_msgs::Vector3(); // linear velocity
  this->number_of_messages = 0;               // number of messages read on /cmd_vel topic
  this->v_x;                                  // previous v_x read on /cmd_vel topic (Vx)
  this->v_y;                                  // previous v_y read on /cmd_vel topic (Vy)
  this->w_z;                                  // previous w_z read on /cmd_vel topic (Wz)
  this->lastTime;                             // time of the last message
  // (x, y, ϑ)
  this->x_k = 0;
  this->y_k = 0;
  this->theta = 0;
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Subscriber::odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  double delta_t;
  // Need at least two messages in order to compute the odometry
  if(number_of_messages != 0){
    double curr_time = msg->header.stamp.sec;
    delta_t = curr_time - this->lastTime;

    auto parameters = geometry_msgs::Vector3();
    switch(this->integMethod){
      case Euler:
        parameters = this->computeEuler(delta_t);
        break;
      case RK:
        parameters = this->computeRungeKutta(delta_t);
        break;
    }
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, parameters.z);

    nav_msgs::Odometry response;

    response.header.frame_id = "world";
    response.header.stamp = ros::Time::now();

    response.pose.pose.position.x = parameters.x;
    response.pose.pose.position.y = parameters.y;
    response.pose.pose.position.z = 0.0;

    response.pose.pose.orientation.x = myQuaternion.getX();
    response.pose.pose.orientation.y = myQuaternion.getY();
    response.pose.pose.orientation.z = myQuaternion.getZ();
    response.pose.pose.orientation.w = myQuaternion.getW();

    this->pub.publish(response);
  }
  this->v_x = msg->twist.linear.x;
  this->v_y = msg->twist.linear.y;
  this->w_z = msg->twist.angular.z;
  this->lastTime = msg->header.stamp.sec;
  this->number_of_messages++;
}

geometry_msgs::Vector3 Subscriber::computeEuler(double delta_t){
  double x_k1 = this->x_k + (v_x * cos(theta) + v_y * cos(theta + M_PI_2)) * delta_t;
  double y_k1 = this->y_k + (v_x * sin(theta) + v_y * sin(theta + M_PI_2)) * delta_t;
  double theta_k1 = this->theta + w_z * delta_t;
  this->x_k = x_k1;
  this->y_k = y_k1;
  this->theta = theta_k1;
  return asVector3(x_k1, y_k1, theta_k1);
}

geometry_msgs::Vector3 Subscriber::computeRungeKutta(double delta_t){
  double coeff = (w_z * delta_t) / 2; //(Wk * Ts) / 2
  double x_k1 = this->x_k + (v_x * cos(theta + coeff) + v_y * cos(theta + M_PI_2 + coeff)) * delta_t;
  double y_k1 = this->y_k + (v_x * sin(theta + coeff) + v_y * sin(theta + M_PI_2 + coeff)) * delta_t;
  double theta_k1 = this->theta + w_z * delta_t;
  this->x_k = x_k1;
  this->y_k = y_k1;
  this->theta = theta_k1;
  return asVector3(x_k1, y_k1, theta_k1);
}