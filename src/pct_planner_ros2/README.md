# PCT Planner for ROS2 Humble

This is the ROS2 Humble version of the PCT (Point Cloud Tomography) Planner, originally designed for ROS Noetic.

## Overview

This package provides a ROS2 port of the PCT Planner, which implements the paper "Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography". The planner is designed for ground robots navigating in multi-layer structures.

## Prerequisites

### Environment
- Ubuntu 22.04
- ROS2 Humble
- Python >= 3.8
- [CuPy](https://docs.cupy.dev/en/stable/install.html) with CUDA >= 11.7
- Open3d

### Installation

1. Install dependencies:
```bash
pip3 install cupy-cuda11x  # or appropriate CUDA version
pip3 install open3d
pip3 install numpy
```

2. Build the C++ libraries (if available):
```bash
cd ~/3d_dog_navi_ros2/PCT_planner/planner/
./build_thirdparty.sh
./build.sh
```

## Build & Install

```bash
cd ~/3d_dog_navi_ros2
colcon build --packages-select pct_planner_ros2
source install/setup.bash
```

## Usage

### Run the planner node:
```bash
ros2 run pct_planner_ros2 pct_planner_node --ros-args -p scene:=Spiral
```

### Using launch file:
```bash
ros2 launch pct_planner_ros2 pct_planner.launch.py scene:=Spiral
```

### Set goals:
You can send goals via:
- RViz2's "2D Nav Goal" tool
- Command line:
```bash
ros2 topic pub /goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Notes

- The original PCT planner used interactive markers which are not directly available in ROS2. This implementation uses standard ROS2 topics for goal setting.
- The C++ libraries need to be recompiled with pybind11 for ROS2 compatibility.
- The package publishes trajectories to the `/pct_path` topic as `nav_msgs/Path` messages.