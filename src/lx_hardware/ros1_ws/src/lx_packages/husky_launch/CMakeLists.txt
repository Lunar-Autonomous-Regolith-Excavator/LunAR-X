cmake_minimum_required(VERSION 3.0.2)
project(husky_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# catkin specific configuration 
catkin_package()

# Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)