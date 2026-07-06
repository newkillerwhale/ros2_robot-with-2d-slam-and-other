cmake_minimum_required(VERSION 3.5)
project(yahboomcar_base_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
)

# Executables
add_executable(base_node_X3 src/base_node_X3.cpp)
add_executable(base_node_x1 src/base_node_x1.cpp)
add_executable(base_node_R2 src/base_node_R2.cpp)

# Dependencies
ament_target_dependencies(base_node_X3 rclcpp tf2 tf2_ros nav_msgs geometry_msgs turtlesim)
ament_target_dependencies(base_node_x1 rclcpp tf2 tf2_ros nav_msgs geometry_msgs turtlesim)
ament_target_dependencies(base_node_R2 rclcpp tf2 tf2_ros nav_msgs geometry_msgs turtlesim)

# Install targets
install(TARGETS
  base_node_X3
  base_node_x1
  base_node_R2
  DESTINATION lib/${PROJECT_NAME}
)

# Package
ament_package()
