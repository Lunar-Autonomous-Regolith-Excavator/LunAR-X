cmake_minimum_required(VERSION 3.8)
project(lx_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

# Task planning node
add_executable(task_planner_node src/task_planner_node.cpp src/task_planner.cpp)
ament_target_dependencies(task_planner_node rclcpp lx_msgs lx_library geometry_msgs tf2_geometry_msgs nav_msgs)

add_executable(task_optimizer_node src/task_optimizer_node.cpp src/task_optimizer.cpp)
ament_target_dependencies(task_optimizer_node rclcpp lx_msgs lx_library geometry_msgs tf2_geometry_msgs nav_msgs)

add_executable(map_pub src/temp_map_pub_node.cpp)
ament_target_dependencies(map_pub rclcpp nav2_msgs)

add_executable(test_planner src/test_task_planner.cpp)
ament_target_dependencies(test_planner rclcpp lx_msgs geometry_msgs)

add_executable(simple-example src/node_se2.cpp src/simple_example.cpp)
target_link_libraries(simple-example ompl ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(gui-example src/node_se2.cpp src/gui_example.cpp)
target_link_libraries(gui-example ompl ceres ${OpenCV_LIBS} Eigen3::Eigen)

# Install python executables
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  src/call_task_optimizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  task_planner_node
  task_optimizer_node
  map_pub
  test_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
