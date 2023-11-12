cmake_minimum_required(VERSION 3.8)
project(lx_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(include)

# Operations handler node
add_executable(operations_handler_node src/operations_handler_node.cpp src/operations_handler.cpp)
ament_target_dependencies(operations_handler_node rclcpp rclcpp_action rclcpp_components geometry_msgs lx_msgs lx_library visualization_msgs)

# Auto Dig Handler node
add_executable(auto_dig_handler_node src/auto_dig_handler_node.cpp src/auto_dig_handler.cpp)
ament_target_dependencies(auto_dig_handler_node rclcpp rclcpp_action rclcpp_components lx_msgs lx_library std_msgs)

# Auto Dump Handler node
add_executable(auto_dump_handler_node src/auto_dump_handler_node.cpp src/auto_dump_handler.cpp)
ament_target_dependencies(auto_dump_handler_node rclcpp rclcpp_action rclcpp_components lx_msgs lx_library)

# Auto Nav Handler node
add_executable(auto_nav_handler_node src/auto_nav_handler_node.cpp src/auto_nav_handler.cpp)
ament_target_dependencies(auto_nav_handler_node rclcpp rclcpp_action rclcpp_components lx_msgs lx_library nav_msgs nav2_msgs tf2_ros)


install(TARGETS
  operations_handler_node
  auto_dig_handler_node
  auto_dump_handler_node
  auto_nav_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
