cmake_minimum_required(VERSION 3.8)
project(lx_status_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Status relay node
add_executable(status_relay_node src/status_relay_node.cpp src/status_relay.cpp)
ament_target_dependencies(status_relay_node rclcpp std_msgs)

install(TARGETS
  status_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()