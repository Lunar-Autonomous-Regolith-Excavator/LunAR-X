cmake_minimum_required(VERSION 3.0.2)
project(husky_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
)

# catkin specific configuration 
catkin_package()

# build send_goal.py
catkin_install_python(PROGRAMS
  src/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)