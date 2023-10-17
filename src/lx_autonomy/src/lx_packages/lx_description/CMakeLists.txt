cmake_minimum_required(VERSION 3.8)
project(lx_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(include)

# Joint visualization node
add_executable(joint_viz_node src/joint_viz_node.cpp src/joint_viz.cpp)
ament_target_dependencies(joint_viz_node rclcpp sensor_msgs)

install(TARGETS
  joint_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_package()
