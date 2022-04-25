#include "first_project/Subscriber.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "first_project/Wheels_RPM.h"
#include "math.h"

#define N 42              //Counts per revolution (CPR)
#define T 5               //Gear ratio
#define WHEEL_RADIUS 0.07 //wheel radius (r)
#define L 0.200           //wheel position along x (l)
#define W 0.169           //wheel position along y (w)

Subscriber::Subscriber() { // class constructor
  //subscriber that listen on "/cmd_vel" topic
  this->sub = this->n.subscribe("/cmd_vel", 1000, &Subscriber::controlCallback, this);
  //publisher that publish a nav_msgs/Odometry message on "/odom" topic
  this->pub = this->n.advertise<first_project::Wheels_RPM>("/wheels_rpm", 1000);
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Subscriber::controlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // read Vx, Vy and Wz from /cmd_vel topic
    double v_x = msg->twist.linear.x;
    double v_y = msg->twist.linear.y;
    double w_z = msg->twist.angular.z;
    // compute the velocities of the four mecanum wheels
    double u1, u2, u3, u4;
    u1 = (((-L-W) * w_z + v_x - v_y) / WHEEL_RADIUS) * 60 * 5;
    u2 = (((+L+W) * w_z + v_x + v_y) / WHEEL_RADIUS) * 60 * 5;
    u3 = (((-L-W) * w_z + v_x + v_y) / WHEEL_RADIUS) * 60 * 5;
    u4 = (((+L+W) * w_z + v_x - v_y) / WHEEL_RADIUS) * 60 * 5;
    // initialize a custom message of type "Wheels_RPM" and publishes it on /wheels_rpm topic
    first_project::Wheels_RPM response;
    response.rpm_fl = u1;
    response.rpm_fr = u2;
    response.rpm_rr = u3;
    response.rpm_rl = u4;

    this->pub.publish(response);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_sub");

  Subscriber my_subscriber;

  my_subscriber.main_loop();

  return 0;
}
