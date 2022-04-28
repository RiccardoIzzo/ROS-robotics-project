#include "first_project/Subscriber.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/calibration_paramConfig.h>
#include "math.h"

#define T 5               //Gear ratio
#define POW 1.0e9

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
  //subscriber that listen on "/wheel_states" topic
  this->sub = this->n.subscribe("/wheel_states", 1000, &Subscriber::velocityCallback, this);
  //publisher that publish a geometry_msgs/TwistStamped message on "/odom" topic
  this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
  // number of messages read on /wheel_states topic
  this->number_of_messages = 0;     
  this->N = 42; 
  this->L = 0.077;  
  this->W = 0.200;  
  this->RADIUS = 0.169;          
}

void Subscriber::main_loop() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void calibration_param_callback(int* N, double* L, double* W, double* RADIUS, first_project::calibration_paramConfig &config, uint32_t level) {
  //ROS_INFO("Integration method: %d - Level %d", config.integMethod, level);
  *N = config.n_param;
  *L = config.l_param;
  *W = config.w_param;
  *RADIUS = config.wheel_radius_param;
}

void Subscriber::velocityCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  double delta_t;
  // Need at least two messages in order to compute the velocities of the wheels
  if(number_of_messages != 0){
    // Compute position of 4 wheels
    double pos_fl = msg->position[0] - this->previous_positions[0];
    double pos_fr = msg->position[1] - this->previous_positions[1];
    double pos_rl = msg->position[2] - this->previous_positions[2];
    double pos_rr = msg->position[3] - this->previous_positions[3];

    //ROS_INFO("Position front left: [%lf]", pos_fl);
    //ROS_INFO("Position front right: [%lf]", pos_fr);
    //ROS_INFO("Position rear left: [%lf]", pos_rl);
    //ROS_INFO("Position rear right: [%lf]", pos_rr);

    double secs = msg->header.stamp.sec - this->previous_time_sec;
    // check if there is a difference in terms of seconds
    if(secs > 0) delta_t = secs - this->previous_time_nsec/POW + msg->header.stamp.nsec/POW;
    else delta_t = msg->header.stamp.nsec/POW - this->previous_time_nsec/POW;
    
    // Compute angular velocities of each wheel
    double w_fl = pos_fl / delta_t / this->N / T * 2.0 * M_PI;
    double w_fr = pos_fr / delta_t / this->N / T * 2.0 * M_PI;
    double w_rl = pos_rl / delta_t / this->N / T * 2.0 * M_PI;
    double w_rr = pos_rr / delta_t / this->N / T * 2.0 * M_PI;

    ROS_INFO("Seq: %d", msg->header.seq);
    ROS_INFO("Velocity front left: [%lf]", w_fl);
    ROS_INFO("Velocity front right: [%lf]", w_fr);
    ROS_INFO("Velocity rear left: [%lf]", w_rl);
    ROS_INFO("Velocity rear right: [%lf]\n", w_rr);
    

    // Prepare to publish a message of type "geometry_msgs::TwistStamped" on /cmd_vel topic
    geometry_msgs::TwistStamped response;

    response.header.frame_id = "";
    response.header.stamp = ros::Time::now();

    // Longitudinal velocity: Vx
    double v_x = (w_fl + w_fr + w_rl + w_rr) * this->RADIUS / 4;

    // Transversal velocity: Vy
    double v_y = (-w_fl + w_fr - w_rr + w_rl) * this->RADIUS / 4;

    // Angular velocity: Wz
    double w_z = (-w_fl + w_fr + w_rr - w_rl) * this->RADIUS / 4 / (this->L + this->W);

    // Compute three components
    response.twist.linear = calcForwardVelocity(v_x, v_y);
    response.twist.angular = calcAngularVelocity(w_z);

    // Publish velocities on "cmd_vel" topic
    this->pub.publish(response);
  }

  this->previous_positions[0] = msg->position[0];
  this->previous_positions[1] = msg->position[1];
  this->previous_positions[2] = msg->position[2];
  this->previous_positions[3] = msg->position[3];
  this->previous_time_sec = msg->header.stamp.sec;
  this->previous_time_nsec = msg->header.stamp.nsec;
  this->number_of_messages++;
}

geometry_msgs::Vector3 Subscriber::calcForwardVelocity(double x, double y) 
{				
	return asVector3(x, y, 0);
}


geometry_msgs::Vector3 Subscriber::calcAngularVelocity(double z)
{
	return asVector3(0, 0, z);	
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_sub");
  
  Subscriber my_subscriber;

  // Dynamic reconfigure of integration method
  dynamic_reconfigure::Server<first_project::calibration_paramConfig> dynServer;
  dynamic_reconfigure::Server<first_project::calibration_paramConfig>::CallbackType f;
  f = boost::bind(&calibration_param_callback, &my_subscriber.N, &my_subscriber.L, &my_subscriber.W, &my_subscriber.RADIUS, _1, _2);
  dynServer.setCallback(f);

  my_subscriber.main_loop();

  return 0;
}

