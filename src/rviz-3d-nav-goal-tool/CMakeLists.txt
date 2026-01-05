cmake_minimum_required(VERSION 3.8)
project(rviz-3d-nav-goal-tool)

# Default to C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

# Qt5 is required for RViz2
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/goal_tool.cpp
  src/pose_tool.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/goal_tool.hpp
  include/${PROJECT_NAME}/pose_tool.hpp
)

# Qt's meta-object compiler
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

# Add library to create plugin
add_library(rviz_3d_nav_goal_tool
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS} 
)

# Include directories
include_directories(
  include
)

target_link_libraries(rviz_3d_nav_goal_tool
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  Qt5::Core
  Qt5::Widgets
)

# Install library
install(TARGETS rviz_3d_nav_goal_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rviz_3d_nav_goal_tool)
ament_package()