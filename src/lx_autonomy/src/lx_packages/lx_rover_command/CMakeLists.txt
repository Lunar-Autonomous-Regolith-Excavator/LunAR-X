cmake_minimum_required(VERSION 3.8)
project(lx_rover_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Command mux node
add_executable(command_mux_node src/command_mux_node.cpp src/command_mux.cpp)
ament_target_dependencies(command_mux_node rclcpp geometry_msgs lx_msgs lx_library)

install(TARGETS
  command_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
