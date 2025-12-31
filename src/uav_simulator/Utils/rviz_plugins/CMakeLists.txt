cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre3d REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(multi_map_server REQUIRED)

# Qt settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(HEADER_FILES
  src/pose_tool.h
  src/goal_tool.h
  src/probmap_display.h
  src/aerialmap_display.h
  src/multi_probmap_display.h
)

set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp   
  src/multi_probmap_display.cpp
)

qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
  src/probmap_display.h
  src/aerialmap_display.h
  src/multi_probmap_display.h
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
  ${MOC_FILES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rviz_common"
  "rviz_ogre3d"
  "rviz_rendering"
  "rviz_default_plugins"
  "multi_map_server"
)

target_link_libraries(${PROJECT_NAME}
  ${Qt5Widgets_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rviz_common Qt5)

ament_package()

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
