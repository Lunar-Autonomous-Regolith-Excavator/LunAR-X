cmake_minimum_required(VERSION 3.8)
project(lx_hardware_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(include)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Calibrate.action"
)

# Hardware mux node
add_executable(hardware_mux_node src/hardware_mux_node.cpp src/hardware_mux.cpp)
ament_target_dependencies(hardware_mux_node rclcpp geometry_msgs lx_msgs action_msgs rclcpp_action rclcpp_components)
# in galactic or earlier
rosidl_target_interfaces(hardware_mux_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# #in humble or later
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(my_node "${cpp_typesupport_target}") 

install(TARGETS
  hardware_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()