cmake_minimum_required(VERSION 3.8)
project(lx_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(lx_library REQUIRED)

include_directories(include)

# Diagnostics node
add_executable(diagnostics_node src/diagnostics_node.cpp src/diagnostics.cpp)
ament_target_dependencies(diagnostics_node rclcpp lx_msgs lx_library)

install(TARGETS
  diagnostics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
