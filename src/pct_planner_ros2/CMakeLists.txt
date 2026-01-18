cmake_minimum_required(VERSION 3.8)
project(pct_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

# Include Eigen3
include_directories(${EIGEN3_INCLUDE_DIRS})

# 创建库
add_library(pct_planner_lib
  src/a_star.cpp
  src/ele_planner.cpp
  src/traj_opt.cpp
)

target_include_directories(pct_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pct_planner_lib
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
)

# 暂时禁用pybind11绑定，先编译C++库
# pybind11_add_module(pct_planner_cpp src/bindings.cpp)
# 
# target_include_directories(pct_planner_cpp PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#   $<INSTALL_INTERFACE:include>)
# 
# # 手动链接库和依赖项
# target_link_libraries(pct_planner_cpp PRIVATE
#   pct_planner_lib
#   pybind11::module
# )
# 
# # 手动添加依赖项
# target_include_directories(pct_planner_cpp PRIVATE
#   ${rclcpp_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${nav_msgs_INCLUDE_DIRS}
# )

# 安装库
install(TARGETS pct_planner_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Python模块
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  pct_planner_ros2/plan.py
  pct_planner_ros2/planner_wrapper.py
  pct_planner_ros2/utils.py
  pct_planner_ros2/config.py
  pct_planner_ros2/tomography.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pct_planner_lib pct_planner_cpp)
ament_export_dependencies()

ament_package()