cmake_minimum_required(VERSION 3.5)
project(h2trac_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RobotStatus.msg"
  "msg/SetMission.msg"
  "msg/SetMotion.msg"
  "msg/MissionStatus.msg"
  "msg/Point2d.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
