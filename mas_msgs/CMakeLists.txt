cmake_minimum_required(VERSION 3.5)
project(mas_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  builtin_interfaces 
  std_msgs
  geometry_msgs
  action_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
#	your custom interfaces will be here
#	one per line
#	no comma for separating lines
	"msg/FleetStatus.msg"
	"msg/Forecast.msg"
	"msg/Status.msg"
	"msg/VehicleStatus.msg"
	"msg/Task.msg"
	"msg/TaskList.msg"
	"msg/VehicleFeedback.msg"
	"msg/CAcommand.msg"
	"msg/Eventpara.msg"
	"srv/Empty.srv"
	"srv/ControlAvailable.srv"
	"srv/GetControl.srv"
	"srv/SetStatus.srv"
	"srv/RouteAvailable.srv"
	"srv/GetRoute.srv"
	"srv/RobotIdle.srv"
	"srv/StartRoute.srv"
	"srv/RouteStarted.srv"
	"srv/RouteFinished.srv"
	"srv/StandardAction.srv"
	"action/Empty.action"
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
