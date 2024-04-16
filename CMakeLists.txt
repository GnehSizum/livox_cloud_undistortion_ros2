cmake_minimum_required(VERSION 3.8)
project(livox_undistortion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  livox_dedistortion_node
  src/livox_dedistortion.cpp
  src/data_process.cpp
  src/gyr_int.cpp
  )

target_include_directories(
  livox_dedistortion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  )

target_link_libraries(
  livox_dedistortion_node 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen
  )

ament_target_dependencies(
  livox_dedistortion_node 
  ${dependencies}
  )

install(TARGETS livox_dedistortion_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()