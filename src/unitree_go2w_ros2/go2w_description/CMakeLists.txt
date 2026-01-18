cmake_minimum_required(VERSION 3.8)
project(go2w_description)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(
  include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# Add library for livox GPU laser plugin
add_library(livox_sim_gpu_laser SHARED
  src/livox_sim_gpu_laser.cpp
)

# Add target-specific include directories
target_include_directories(livox_sim_gpu_laser PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# Add ROS2 message include directories
ament_target_dependencies(livox_sim_gpu_laser
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Link libraries
target_link_libraries(livox_sim_gpu_laser
  ${gazebo_dev_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

# Add compile definitions
target_compile_definitions(livox_sim_gpu_laser PRIVATE
  BOOST_ALL_NO_LIB
  GAZEBO_GPU_RAY
)

# Install library
install(TARGETS
  livox_sim_gpu_laser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install resource directories
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
    models
    scan_mode
  DESTINATION share/${PROJECT_NAME}
)

# Install include headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# ament package macro
ament_package()
