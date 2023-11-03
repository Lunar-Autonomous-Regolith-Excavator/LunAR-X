cmake_minimum_required(VERSION 3.8)
project(lx_external_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# External interface node
add_executable(external_interface_node src/external_interface_node.cpp src/external_interface.cpp)
ament_target_dependencies(external_interface_node rclcpp sensor_msgs lx_msgs lx_library)

# Goal handler node
add_executable(goal_handler_node src/goal_handler_node.cpp src/goal_handler.cpp)
ament_target_dependencies(goal_handler_node rclcpp lx_msgs lx_library geometry_msgs visualization_msgs)

install(TARGETS
  external_interface_node
  goal_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
