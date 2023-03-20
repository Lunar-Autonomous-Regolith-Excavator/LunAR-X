cmake_minimum_required(VERSION 3.8)
project(lx_hardware_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Hardware mux node
add_executable(hardware_mux_node src/hardware_mux_node.cpp src/hardware_mux.cpp)
ament_target_dependencies(hardware_mux_node rclcpp geometry_msgs lx_msgs)

install(TARGETS
  hardware_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()