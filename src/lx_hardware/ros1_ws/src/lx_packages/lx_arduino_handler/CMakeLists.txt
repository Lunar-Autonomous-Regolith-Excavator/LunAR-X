cmake_minimum_required(VERSION 3.0.2)
project(lx_arduino_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# catkin specific configuration 
catkin_package()

#build rospy node
catkin_install_python(PROGRAMS src/arduino_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)