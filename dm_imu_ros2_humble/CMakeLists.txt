cmake_minimum_required(VERSION 3.8)
project(dm_imu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(
  include
)

add_executable(imu_node
  src/imu_driver.cpp
  src/imu_node.cpp
  src/bsp_crc.cpp
)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  serial
)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
