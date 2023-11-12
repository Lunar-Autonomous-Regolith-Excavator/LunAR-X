cmake_minimum_required(VERSION 3.5)
project(lx_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

include_directories(include)

#remap node for husky odom
add_executable(localization_node src/localization_node.cpp src/localization.cpp)
ament_target_dependencies(localization_node rclcpp nav_msgs sensor_msgs tf2 tf2_geometry_msgs geometry_msgs lx_msgs rclcpp_action visualization_msgs)

install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
