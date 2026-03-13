cmake_minimum_required(VERSION 3.5)
project(bt_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(dependencies
  builtin_interfaces 
  std_msgs
  action_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
#	your custom interfaces will be here
#	one per line
#	no comma for separating lines
	"msg/Empty.msg"
	"msg/Param.msg"
	"srv/Empty.srv"
	"srv/AddTwoInts.srv"
  "srv/LocationServer.srv"
	"action/Empty.action"
	"action/Fibonacci.action"
	"action/PlayRole.action"
  "action/RoleLelyLa.action"
  "action/RoleLelyCa.action"
  "action/RoleLelyVsa.action"
  "action/RoleLelyVoa.action"
  "action/RoleLelyFa.action"
  "action/RoleLelyMla.action"
	DEPENDENCIES ${dependencies}
)

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

ament_export_dependencies(${dependencies})

ament_package()
