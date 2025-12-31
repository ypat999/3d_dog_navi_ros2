cmake_minimum_required(VERSION 3.8)
project(livox_gazebo_ros2_gpu_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# cpu mid360
find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)

# Get gazebo_ros install prefix
get_filename_component(GAZEBO_ROS_PREFIX "${gazebo_ros_DIR}/../../../" ABSOLUTE)

# Find Gazebo and SDF
find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED gazebo)
pkg_check_modules(SDFORMAT REQUIRED sdformat9)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Include directories
include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${SDFORMAT_INCLUDE_DIRS})

# Link directories for Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create the GPU laser plugin library
add_library(livox_sim_gpu_laser SHARED
  src/livox_sim_gpu_laser.cpp
)

# Create test executable
add_executable(laser_listener
  src/laser_listener.cpp
)

# Link libraries
ament_target_dependencies(livox_sim_gpu_laser
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  ament_index_cpp
)

target_link_libraries(livox_sim_gpu_laser
  ${GAZEBO_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  GpuRayPlugin
  OpenMP::OpenMP_CXX
)

# Add OpenMP compile flags if needed
if(OpenMP_CXX_FOUND)
  target_compile_options(livox_sim_gpu_laser PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# Link test executable
ament_target_dependencies(laser_listener
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  livox_sim_gpu_laser
  laser_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



#cpu mid360
add_library(ros2_livox SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(ros2_livox ${GAZEBO_LIBRARIES} RayPlugin GpuRayPlugin)
ament_target_dependencies(ros2_livox rclcpp std_msgs sensor_msgs geometry_msgs gazebo_dev gazebo_ros tf2_ros livox_ros_driver2 )
target_link_libraries(ros2_livox libprotobuf.so.9)
target_link_libraries(ros2_livox libboost_chrono.so.1.71.0)
target_include_directories(ros2_livox PRIVATE include)

#install(TARGETS ros2_livox DESTINATION lib/${PROJECT_NAME})

install(TARGETS ros2_livox
    LIBRARY DESTINATION lib)

ament_export_libraries(
  ros2_livox
)



# Install header files
install(DIRECTORY include/
  DESTINATION include
)



# Install rviz configuration files
install(
  DIRECTORY launch rviz urdf worlds meshes scan_mode config
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB PYTHON_SCRIPTS 
  "${CMAKE_CURRENT_SOURCE_DIR}/python_scripts/*.py"  
)

# Install Python scripts with symlink support for colcon --symlink-install
if(AMENT_CMAKE_SYMLINK_INSTALL)
  ament_cmake_symlink_install_files(FILES ${PYTHON_SCRIPTS}
    DESTINATION "lib/${PROJECT_NAME}"
  )
else()
  install(
    FILES ${PYTHON_SCRIPTS}  # 要安装的文件：所有Python脚本
    DESTINATION lib/${PROJECT_NAME}  # 安装目标路径（ROS 2标准，PROJECT_NAME即你的包名）
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ  # 所有者权限（执行/写/读）
                GROUP_EXECUTE GROUP_READ              # 组用户权限（执行/读）
                WORLD_EXECUTE WORLD_READ              # 其他用户权限（执行/读）
  )
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Set up environment hooks to add lib directory to GAZEBO_PLUGIN_PATH
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/livox_sim_gpu_ros2.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/env-hooks/livox_sim_gpu_ros2.sh"
  @ONLY
)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/env-hooks/livox_sim_gpu_ros2.sh")

ament_package()