cmake_minimum_required(VERSION 3.8)
project(lx_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)

include_directories(include)

# Task planning node
add_executable(task_planner_node src/task_planner_node.cpp src/task_planner.cpp)
ament_target_dependencies(task_planner_node rclcpp lx_msgs lx_library)

install(TARGETS
task_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
