cmake_minimum_required(VERSION 3.5)
project(lx_mapping)

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
find_package(lx_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(berm_evaluation_node src/berm_evaluation_node.cpp src/berm_evaluation.cpp)
ament_target_dependencies(berm_evaluation_node rclcpp lx_msgs rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_eigen tf2 pcl_ros pcl_conversions)

add_executable(merge_map_node src/merge_map_node.cpp src/merge_map.cpp)
ament_target_dependencies(merge_map_node pcl_ros rclcpp tf2_geometry_msgs lx_msgs rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_eigen tf2 pcl_ros pcl_conversions)

add_executable(pcl_relay_node src/pcl_relay_node.cpp src/pcl_relay.cpp)
ament_target_dependencies(pcl_relay_node rclcpp sensor_msgs)

install(TARGETS 
  berm_evaluation_node
  pcl_relay_node
  merge_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(merge_map_node ${PCL_LIBRARIES})

ament_package()
