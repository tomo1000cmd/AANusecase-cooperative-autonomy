cmake_minimum_required(VERSION 3.5)
project(fa)

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
)

## Cpp executables

set(executable_name fa_bt)

add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies} 
)

target_link_libraries(${executable_name} ${library_name})

## Cpp Plugins


##-------------------------------------------------------ACTIONS----------------------------------------------
add_library(compute_send_service_service_bt_node SHARED plugins/action/compute_send_service.cpp)
list(APPEND plugin_libs compute_send_service_service_bt_node)
##-------------------------------------------------------CODITIONS---------------------------------------------------------
add_library(is_message_fleet_get_fa_condition_bt_node SHARED plugins/condition/is_fleet_message_get_condition.cpp)
list(APPEND plugin_libs is_message_fleet_get_fa_condition_bt_node)

add_library(is_request_get_condition_condition_bt_node SHARED plugins/condition/is_request_get_condition.cpp)
list(APPEND plugin_libs is_request_get_condition_condition_bt_node)
##-----------------------------------------------------------------------------------------------------------------------
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
  scripts/fa_bt_handler_h2.py
	scripts/fa.py
  scripts/fa_bt_handler.py
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
  maps
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
