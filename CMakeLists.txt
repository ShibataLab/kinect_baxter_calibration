cmake_minimum_required(VERSION 2.8.3)
project(kinect_baxter_calibration)

find_package(
  catkin
  REQUIRED
  COMPONENTS
  roscpp
  rostime
  rosbag
  std_msgs)

## catkin python configuration ##
catkin_python_setup()

## catkin configuration ##
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS)

## build ##
#include_directories(include ${catkin_INCLUDE_DIRS})

## install ##
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} USE_SOURCE_PERMISSIONS)
#install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} USE_SOURCE_PERMISSIONS)
