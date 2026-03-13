cmake_minimum_required(VERSION 3.5)
project(voa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bt_agent REQUIRED)
find_package(bt_msgs REQUIRED)
find_package(agent_msgs REQUIRED)
find_package(lely_msgs REQUIRED)
find_package(h2trac_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  builtin_interfaces
  std_msgs
  behaviortree_cpp_v3
  nav2_msgs
	nav2_util
  tf2
  tf2_ros
  bt_agent
  bt_msgs
  agent_msgs
  lely_msgs
  h2trac_msgs
)

## Cpp executables

set(executable_name voa_bt)

add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies} 
)

target_link_libraries(${executable_name} ${library_name})

## Cpp Plugins
##----------------------------------------------------------Conditions
add_library(robot_idle_condition_bt_node SHARED plugins/condition/robot_idle_condition.cpp)
list(APPEND plugin_libs robot_idle_condition_bt_node)

add_library(control_available_condition_bt_node SHARED plugins/condition/control_available_condition.cpp)
list(APPEND plugin_libs control_available_condition_bt_node)

add_library(route_available_condition_bt_node SHARED plugins/condition/route_available_condition.cpp)
list(APPEND plugin_libs route_available_condition_bt_node)

add_library(route_finished_condition_bt_node SHARED plugins/condition/route_finished_condition.cpp)
list(APPEND plugin_libs route_finished_condition_bt_node)

add_library(route_started_condition_bt_node SHARED plugins/condition/route_started_condition.cpp)
list(APPEND plugin_libs route_started_condition_bt_node)
#------------------------------------------------------------------------------------
add_library(robot_available_condition_bt_node SHARED plugins/condition/robot_available_condition.cpp)
list(APPEND plugin_libs robot_available_condition_bt_node)

add_library(is_time_passed_condition_bt_node SHARED plugins/condition/is_time_passed_condition.cpp)
list(APPEND plugin_libs is_time_passed_condition_bt_node)

add_library(is_event_get_condition_bt_node SHARED plugins/condition/is_event_get_condition.cpp)
list(APPEND plugin_libs is_event_get_condition_bt_node)

add_library(is_CAcommand_get_condition_bt_node SHARED plugins/condition/is_CAcommand_get_condition.cpp)
list(APPEND plugin_libs is_CAcommand_get_condition_bt_node)

#------------------------------------------------------------------------------------Services
add_library(get_control_service_bt_node SHARED plugins/action/get_control_service.cpp)
list(APPEND plugin_libs get_control_service_bt_node)

add_library(get_route_service_bt_node SHARED plugins/action/get_route_service.cpp)
list(APPEND plugin_libs get_route_service_bt_node)

add_library(start_route_service_bt_node SHARED plugins/action/start_route_service.cpp)
list(APPEND plugin_libs start_route_service_bt_node)

add_library(set_status_service_bt_node SHARED plugins/action/set_status_service.cpp)
list(APPEND plugin_libs set_status_service_bt_node)

#------------------------------------------------------------------------------------
add_library(send_status_service_bt_node SHARED plugins/action/send_status_service.cpp)
list(APPEND plugin_libs send_status_service_bt_node)

add_library(update_status_service_bt_node SHARED plugins/action/update_status_service.cpp)
list(APPEND plugin_libs update_status_service_bt_node)

add_library(execute_command_service_bt_node SHARED plugins/action/execute_command_service.cpp)
list(APPEND plugin_libs execute_command_service_bt_node)
#------------------------------------------------------------------------------------

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

## Install Cpp executables

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install Cpp libraries and plugins

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install Python modules

ament_python_install_package(${PROJECT_NAME})

## Install Python executables

install(PROGRAMS
	scripts/voa.py
  scripts/voa_command_handler.py
  scripts/voa_bt_handler.py
  scripts/voa_bt_handler_h2.py
  scripts/rcl_harvester.py
  scripts/rcl_chaserbin.py
  scripts/voa_node.py
  DESTINATION lib/${PROJECT_NAME}
)

## Install directories

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

## Install files

# install(FILES some_file DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_package()
