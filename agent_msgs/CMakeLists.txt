cmake_minimum_required(VERSION 3.5)
project(agent_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/DriveOrder.msg"
  "msg/OperationControl.msg"
  "msg/VehicleStatus.msg"
  "msg/CAcommand.msg"
  "msg/Status.msg"
  "msg/Task.msg"
)

set(srv_files
  "srv/RouteAvailable.srv"
  "srv/GetRoute.srv"
  "srv/ControlAvailable.srv"
  "srv/GetControl.srv"
  "srv/StandardAction.srv"
  "srv/StartRoute.srv"
  "srv/SetStatus.srv"
  "srv/RobotIdle.srv"
  "srv/RouteStarted.srv"
  "srv/RouteFinished.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
