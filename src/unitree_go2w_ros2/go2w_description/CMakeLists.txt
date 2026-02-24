cmake_minimum_required(VERSION 3.8)
project(go2w_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

option(BUILD_GAZEBO_CLASSIC_PLUGIN "Build Gazebo Classic plugin" OFF)

if(BUILD_GAZEBO_CLASSIC_PLUGIN)
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GAZEBO REQUIRED gazebo)
  pkg_check_modules(SDFORMAT REQUIRED sdformat9)

  find_package(OpenMP REQUIRED)

  include_directories(include)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  include_directories(${SDFORMAT_INCLUDE_DIRS})

  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(livox_sim_gpu_laser SHARED
    src/livox_sim_gpu_laser.cpp
  )

  target_include_directories(livox_sim_gpu_laser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFORMAT_INCLUDE_DIRS}
  )

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

  if(OpenMP_CXX_FOUND)
    target_compile_options(livox_sim_gpu_laser PRIVATE ${OpenMP_CXX_FLAGS})
  endif()

  target_compile_definitions(livox_sim_gpu_laser PRIVATE
    BOOST_ALL_NO_LIB
    GAZEBO_GPU_RAY
  )

  install(TARGETS
    livox_sim_gpu_laser
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

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

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
