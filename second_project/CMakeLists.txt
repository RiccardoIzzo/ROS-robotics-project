cmake_minimum_required(VERSION 3.0.2)
project(second_project)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  message_generation
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MapSaver.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})



add_executable(odom_tf src/odom_tf.cpp)
add_dependencies(odom_tf ${${second_project}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf ${catkin_LIBRARIES})

add_executable(trajectory_class src/trajectory.cpp)
add_dependencies(trajectory_class ${${second_project}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_class ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
