cmake_minimum_required(VERSION 3.8)
project(lx_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

# Task planning node
add_executable(task_planner_node src/task_planner_node.cpp src/task_planner.cpp)
ament_target_dependencies(task_planner_node rclcpp lx_msgs lx_library geometry_msgs)

add_executable(map_pub src/temp_map_pub_node.cpp)
ament_target_dependencies(map_pub rclcpp nav2_msgs)

add_executable(call_plan src/planner_action_client.cpp)
ament_target_dependencies(call_plan rclcpp rclcpp_action nav2_msgs lx_msgs)

install(TARGETS
  task_planner_node
  map_pub
  call_plan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
