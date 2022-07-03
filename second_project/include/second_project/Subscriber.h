#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/MapMetaData.h"
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

// Subscriber class
class Subscriber {
public:
  Subscriber(); 
  void main_loop();
  void trajectoryCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 
  void mapDataSaverCallback(const nav_msgs::MapMetaData::ConstPtr& msg); 


  // global parameters: must be visible to both the launch file and the service callback function in Odometry.cpp
  ros::NodeHandle n;

private:
  ros::Subscriber amcl_sub, map_sub;
  ros::Publisher pub;
  long int number_of_messages;

  int image_height;
  int image_width;
  float image_resolution;
  float image_origin_x;
  float image_origin_y;
};

#endif
