cmake_minimum_required(VERSION 3.8)
project(go2w_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/high_level_publisher_IHM.py
  src/joint_publisher_IHM.py
  src/hybrid_motion_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()
