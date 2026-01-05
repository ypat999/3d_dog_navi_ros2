cmake_minimum_required(VERSION 3.8)
project(a1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python launch files
install(PROGRAMS
  launch/a1_rviz_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/XACRO files
install(DIRECTORY urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install scan mode files
install(DIRECTORY scan_mode
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
