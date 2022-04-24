#include "ros/ros.h"
#include "first_project/Subscriber.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_sub");


  Subscriber my_subscriber;


  my_subscriber.main_loop();

  return 0;
}
