cmake_minimum_required(VERSION 3.8)
project(lx_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverCommand.msg"
  "msg/ToolInfo.msg"
  "msg/BermProgress.msg"
  "msg/BermConfig.msg"
  "msg/PlannedTask.msg"
  "msg/NodeDiagnostics.msg"
  "msg/BermSection.msg"
  "srv/BermProgressEval.srv"
  "srv/Switch.srv"
  "srv/Plan.srv"
  "srv/UserInputRecord.srv"
  "srv/BermService.srv"
  "srv/ZoneService.srv"
  "srv/RequestRoverService.srv"
  "srv/PclGroundHeight.srv"
  "action/Operation.action"
  "action/AutoDig.action"
  "action/AutoDump.action"
  "action/AutoNav.action"
  "action/CalibrateImu.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs builtin_interfaces
)

ament_package()