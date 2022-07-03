#include "second_project/Subscriber.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "second_project/MapSaver.h"
#include <vector>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

typedef cv::Point_<float> Point2f;
std::vector<Point2f> points;

Subscriber::Subscriber() { // class constructor
  this->amcl_sub = this->n.subscribe("/amcl_pose", 1000, &Subscriber::trajectoryCallback, this);
  this->map_sub = this->n.subscribe("/map_metadata", 1000, &Subscriber::mapDataSaverCallback, this);

  this->image_height;
  this->image_width;
  this->image_resolution;
  this->image_origin_x;
  this->image_origin_y;
}

void Subscriber::trajectoryCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  Point2f p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  points.push_back(p);
  this->number_of_messages++;
}

void Subscriber::mapDataSaverCallback(const nav_msgs::MapMetaData::ConstPtr& msg){
  ROS_INFO("READ MAP METADATA");
  this->image_height = msg->height;
  this->image_width = msg->width;
  this->image_resolution = msg->resolution;
  this->image_origin_x = msg->origin.position.x;
  this->image_origin_y = msg->origin.position.y;
}

bool mapServiceCallback(float *image_origin_x, float *image_origin_y, float *image_resolution, int *image_height, second_project::MapSaver::Request &req, second_project::MapSaver::Response &res){
  ROS_INFO("/MAP SAVER SERVICE CALLED");
  cv::Scalar color(0, 0, 255);   
  auto pgm = cv::imread(ros::package::getPath("second_project")+"/map.pgm", cv::IMREAD_COLOR);
  points[0].x = (points[0].x - *image_origin_x)/(*image_resolution);
  points[0].y = *image_height - (points[0].y - *image_origin_y)/(*image_resolution);
  for(int i = 1; i < points.size(); i++){
    points[i].x = (points[i].x - *image_origin_x)/(*image_resolution);
    points[i].y = *image_height - (points[i].y - *image_origin_y)/(*image_resolution);
    cv::line(pgm, points[i - 1], points[i], color);
  }
  cv::imwrite(ros::package::getPath("second_project")+"/map_with_trajectory.pgm", pgm);
  return true;
}

void Subscriber::main_loop() {
  ros::ServiceServer service = 
    this->n.advertiseService<second_project::MapSaver::Request, second_project::MapSaver::Response>("map_saver", boost::bind(&mapServiceCallback, &this->image_origin_x, &this->image_origin_y, &this->image_resolution, &this->image_height, _1, _2)
  );
  ros::Rate loop_rate(10);
  ros::spin();
}

int main(int argc, char **argv){   
  ros::init(argc, argv, "trajectory_sub");
  Subscriber my_subscriber;
  my_subscriber.main_loop();
  return 0;
}