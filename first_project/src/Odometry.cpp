#include "first_project/Subscriber.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "first_project/Reset.h"
#include <first_project/paramConfig.h>
#include <dynamic_reconfigure/server.h>
#include "math.h"

#define POW 1.0e9
#define Z 0.25

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

  this->integMethod = 0;                      //by default the integration method is Euler (0: Euler, 1: RK)
  this->number_of_messages = 0;               // number of messages read on /cmd_vel topic
}

// reset the robot pose with the values specified in terminal with "rosservice call /reset ..."
bool reset_callback(double *x_k, double *y_k, double *theta, long int *number_of_messages, first_project::Reset::Request &req, first_project::Reset::Response &res) {
  res.old_x = *x_k;
  *x_k = req.new_x;

  res.old_y = *y_k;
  *y_k = req.new_y;

  res.old_theta = *theta;
  *theta = req.new_theta;

  *number_of_messages = 0;

  ROS_INFO("Request to reset pose to [%lf, %lf, %lf] - Responding with old pose: [%lf, %lf, %lf]",
  (double)req.new_x, (double)req.new_y, (double)req.new_theta, (double)res.old_x, (double)res.old_y, (double)res.old_theta);
  return true;
}

// change the integration method with "rosrun rqt_reconfigure rqt_reconfigure"
void param_callback(int* integMethod, first_project::paramConfig &config, uint32_t level) {
  ROS_INFO("Integration method: %d - Level %d", config.integMethod, level);
  *integMethod = config.integMethod;
}

// manage the odometry with two different integration methods (Euler, Runge-Kutta)
void Subscriber::odometryCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  double delta_t;
  // Need at least two messages in order to compute the odometry
  if(number_of_messages != 0){
    double curr_time = msg->header.stamp.sec + msg->header.stamp.nsec/POW;
    delta_t = curr_time - this->lastTime;

    auto parameters = geometry_msgs::Vector3();
    switch(integMethod){
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

    response.header.frame_id = "odom";
    //response.header.child_frame_id = "base_link";
    response.header.stamp = ros::Time::now();

    response.pose.pose.position.x = parameters.x;
    response.pose.pose.position.y = parameters.y;
    response.pose.pose.position.z = Z;

    response.pose.pose.orientation.x = myQuaternion.getX();
    response.pose.pose.orientation.y = myQuaternion.getY();
    response.pose.pose.orientation.z = myQuaternion.getZ();
    response.pose.pose.orientation.w = myQuaternion.getW();

    this->pub.publish(response);
  }
  this->v_x = msg->twist.linear.x;
  this->v_y = msg->twist.linear.y;
  this->w_z = msg->twist.angular.z;
  this->lastTime = msg->header.stamp.sec + msg->header.stamp.nsec/POW ;
  this->number_of_messages++;
}

void Subscriber::main_loop() {
  double init_x;
  double init_y;
  double init_theta;

  // get parameters from launch file (launch.launch)
  this->n.getParam("init_x", init_x);
  this->n.getParam("init_y", init_y);
  this->n.getParam("init_theta", init_theta);

  // initialize the parameters with the initial value described in the launch file
  this->x_k = init_x;
  this->y_k = init_y;
  this->theta = init_theta;

  //start the service "/reset" that allow to reset the odometry to any given pose (x, y, ϑ)
  ros::ServiceServer service = 
    n.advertiseService<first_project::Reset::Request, first_project::Reset::Response>("reset", boost::bind(&reset_callback, &this->x_k, &this->y_k, &this->theta, &this->number_of_messages, _1, _2) 
  );

  ros::Rate loop_rate(10);

  ros::spin();
}

// compute Euler integration
geometry_msgs::Vector3 Subscriber::computeEuler(double delta_t){
  double x_k1 = this->x_k + (v_x * cos(theta) + v_y * cos(theta + M_PI_2)) * delta_t;
  double y_k1 = this->y_k + (v_x * sin(theta) + v_y * sin(theta + M_PI_2)) * delta_t;
  double theta_k1 = this->theta + w_z * delta_t;
  this->x_k = x_k1;
  this->y_k = y_k1;
  this->theta = theta_k1;
  return asVector3(x_k1, y_k1, theta_k1);
}

// compute Runge-Kutta integration
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_sub");

  Subscriber my_subscriber;

  // Dynamic reconfigure of integration method
  dynamic_reconfigure::Server<first_project::paramConfig> dynServer;
  dynamic_reconfigure::Server<first_project::paramConfig>::CallbackType f;
  f = boost::bind(&param_callback, &my_subscriber.integMethod, _1, _2);
  dynServer.setCallback(f);

  my_subscriber.main_loop();

  return 0;
}