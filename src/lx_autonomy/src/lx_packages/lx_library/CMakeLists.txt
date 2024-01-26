cmake_minimum_required(VERSION 3.8)
project(lx_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lx_msgs REQUIRED)

include_directories(include)

add_library(lx_library src/lx_utils.cpp src/task.cpp)
ament_export_targets(lx_library HAS_LIBRARY_TARGET)
ament_target_dependencies(lx_library rclcpp geometry_msgs lx_msgs)

install(
  DIRECTORY include/lx_library
  DESTINATION include
)

install(
  TARGETS lx_library
  EXPORT lx_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
