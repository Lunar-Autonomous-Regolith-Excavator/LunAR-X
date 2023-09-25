cmake_minimum_required(VERSION 3.8)
project(lx_bringup_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)

include_directories(include)

add_executable(param_server_node src/param_server_node.cpp src/param_server.cpp)
ament_target_dependencies(param_server_node rclcpp lx_msgs)

install(TARGETS
  param_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
