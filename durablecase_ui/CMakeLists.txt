cmake_minimum_required(VERSION 3.16)
project(durablecase_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Qt5 requirements
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(supervisor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)

# Header files
set(HEADERS
  include/durablecase_ui/main_window.hpp
  include/durablecase_ui/map_visualization.hpp
)

# Source files
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/map_visualization.cpp
)

# Create executable
add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  supervisor_msgs
  std_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  OpenGL::OpenGL
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
