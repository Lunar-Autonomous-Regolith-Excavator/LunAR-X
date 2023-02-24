cmake_minimum_required(VERSION 3.8)
project(lx_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(lx_utils src/lx_utils.cpp)
ament_export_targets(lx_utils HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/lx_utils
  DESTINATION include
)

install(
  TARGETS lx_utils
  EXPORT lx_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
