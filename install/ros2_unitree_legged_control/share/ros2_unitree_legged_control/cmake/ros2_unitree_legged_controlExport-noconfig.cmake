#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_unitree_legged_control::ros2_unitree_legged_control" for configuration ""
set_property(TARGET ros2_unitree_legged_control::ros2_unitree_legged_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ros2_unitree_legged_control::ros2_unitree_legged_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ros2_unitree_legged_control/libros2_unitree_legged_control.so"
  IMPORTED_SONAME_NOCONFIG "libros2_unitree_legged_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros2_unitree_legged_control::ros2_unitree_legged_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros2_unitree_legged_control::ros2_unitree_legged_control "${_IMPORT_PREFIX}/lib/ros2_unitree_legged_control/libros2_unitree_legged_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
