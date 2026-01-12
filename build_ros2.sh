#!/bin/bash
# ROS2 Packages
if [ -x "$(command -v ./src/livox_ros_driver2/build.sh)" ]; then
    ./src/livox_ros_driver2/build.sh humble
else
    echo "[livox_ros_driver2/build.sh] not found"
    exit 1
fi

cd ./ros2

colcon build --symlink-install --parallel-workers 8
source install/setup.bash
