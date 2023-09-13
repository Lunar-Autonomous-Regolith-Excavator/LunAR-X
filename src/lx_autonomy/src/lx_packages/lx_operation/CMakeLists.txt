cmake_minimum_required(VERSION 3.8)
project(lx_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# Operations handler node
add_executable(operations_handler_node src/operations_handler_node.cpp src/operations_handler.cpp)
ament_target_dependencies(operations_handler_node rclcpp rclcpp_action rclcpp_components geometry_msgs lx_msgs lx_library)

# # Autodig node
# add_executable(autodig_node src/autodig_node.cpp src/autodig.cpp)
# ament_target_dependencies(autodig_node rclcpp lx_msgs lx_library)

# # Autodump node
# add_executable(autodump_node src/autodump_node.cpp src/autodump.cpp)
# ament_target_dependencies(autodump_node rclcpp lx_msgs lx_library)

# # Rover navigate node
# add_executable(rover_navigate_node src/rover_navigate_node.cpp src/rover_navigate.cpp)
# ament_target_dependencies(rover_navigate_node rclcpp lx_msgs lx_library)

install(TARGETS
  operations_handler_node
  # autodig_node
  # autodump_node
  # rover_navigate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
