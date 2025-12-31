# Livox GPU Laser Simulation ROS2 Plugin

A ROS2 Gazebo GPU laser plugin specifically designed for Livox LiDAR simulation. Provides high-performance point cloud simulation based on GPU.

> **Note**: Currently only fully tested on **Mid-360**. Configuration files for other models (Avia, HAP, Horizon, Mid-40, Mid-70, Tele) are included but not thoroughly tested.

[中文文档](README_zh.md)

## Features

- High-performance point cloud simulation based on Gazebo GPU Ray sensor
- Publishes `sensor_msgs/PointCloud2` messages
- Supports scan patterns for multiple Livox LiDAR models
- Customizable topic names, frames, and namespaces
- Pre-configured URDF/Xacro files and 3D models included

## Supported Models

| Model | Scan Pattern File | URDF File | Test Status |
|-------|------------------|-----------|-------------|
| **Mid-360** | `scan_mode/mid360.csv` | `urdf/mid360.xacro` | ✅ Tested |
| Avia | `scan_mode/avia.csv` | `urdf/avia.xacro` | ⚠️ Untested |
| HAP | `scan_mode/HAP.csv` | `urdf/HAP.xacro` | ⚠️ Untested |
| Horizon | `scan_mode/horizon.csv` | `urdf/horizon.xacro` | ⚠️ Untested |
| Mid-40 | `scan_mode/mid40.csv` | `urdf/mid40.xacro` | ⚠️ Untested |
| Mid-70 | `scan_mode/mid70.csv` | `urdf/mid70.xacro` | ⚠️ Untested |
| Tele-15 | `scan_mode/tele.csv` | `urdf/tele.xacro` | ⚠️ Untested |

## Dependencies

### System Requirements
- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Gazebo 11
- SDFormat 9

### ROS2 Package Dependencies
```bash
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-tf2-ros \
  ros-humble-ament-cmake
```

### Build Dependencies
- OpenMP
- gazebo_dev
- pkg-config

## Installation and Build

### 1. Clone Repository

```bash
cd ~/your_workspace/src
git clone git@github.com:ttwards/livox_gazebo_ros2_gpu_simulation livox_sim_gpu_ros2
```

### 2. Install Dependencies

```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
cd ~/your_workspace
colcon build --packages-select livox_gazebo_ros2_gpu_simulation
```

### 4. Setup Environment

```bash
source install/setup.bash
```

After building, the environment hook will automatically set `GAZEBO_PLUGIN_PATH` to add the plugin path to Gazebo's search path.

## Usage

### Using Pre-configured Xacro Files

The project includes pre-configured Xacro files that can be directly referenced in your robot URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- Include Mid-360 LiDAR -->
  <xacro:include filename="$(find livox_gazebo_ros2_gpu_simulation)/urdf/mid360.xacro"/>
  
  <!-- Attach the LiDAR to a robot link -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="mid360_base"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  
</robot>
```

### Available Xacro Files

- `urdf/mid360.xacro` - Mid-360 (Tested)
- `urdf/avia.xacro` - Avia
- `urdf/horizon.xacro` - Horizon
- `urdf/mid40.xacro` - Mid-40
- `urdf/mid70.xacro` - Mid-70
- `urdf/HAP.xacro` - HAP
- `urdf/tele.xacro` - Tele-15

### Plugin Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ray_count` | int | 24000 | Number of rays per frame |
| `min_range` | double | 0.1 | Minimum range (meters) |
| `max_range` | double | 200.0 | Maximum range (meters) |
| `topic` | string | "pointcloud" | ROS2 topic name |
| `frame_id` | string | "lidar_link" | TF frame name |
| `update_rate` | double | 10.0 | Update frequency (Hz) |
| `downsample` | int | 1 | Downsample factor (1=no downsampling) |
| `csv_file_name` | string | "mid360" | Scan pattern CSV filename (without extension) |

### Running Simulation

```bash
# Make sure workspace is sourced
source ~/your_workspace/install/setup.bash

# Launch Gazebo
gazebo your_world.world

# Or use the provided launch file
ros2 launch livox_gazebo_ros2_gpu_simulation test_gpu_laser.launch.py world_file:=/path/to/your.world
```

### Viewing Data

```bash
# List point cloud topics
ros2 topic list | grep pointcloud

# Echo point cloud data
ros2 topic echo /pointcloud

# Visualize in RViz2
rviz2
# Add PointCloud2 display and subscribe to /pointcloud topic
```

## File Structure

```
livox_sim_gpu_ros2/
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS2 package description
├── README.md                   # This file (English)
├── README_zh.md                # Chinese documentation
├── livox_sim_gpu_laser.xml     # Plugin description
├── include/
│   └── livox_sim_plugins/
│       └── livox_sim_gpu_laser.h    # Plugin header
├── src/
│   ├── livox_sim_gpu_laser.cpp      # Plugin implementation
│   └── laser_listener.cpp           # Test listener node
├── launch/
│   └── test_gpu_laser.launch.py     # Test launch file
├── urdf/                       # URDF/Xacro model files
│   ├── mid360.xacro
│   ├── avia.xacro
│   ├── horizon.xacro
│   ├── mid40.xacro
│   ├── mid70.xacro
│   ├── HAP.xacro
│   └── tele.xacro
├── scan_mode/                  # Scan pattern CSV files
│   ├── mid360.csv
│   ├── avia.csv
│   ├── horizon.csv
│   ├── mid40.csv
│   ├── mid70.csv
│   ├── HAP.csv
│   └── tele.csv
├── meshes/
│   └── mid-360-scaled.dae      # 3D model
└── env-hooks/
    └── livox_sim_gpu_ros2.sh.in # Environment setup hook
```

## Notes

1. **GPU Support**: This plugin requires GPU support. If your system doesn't have a GPU or Gazebo cannot initialize the rendering engine, the sensor will not work.
2. **Performance**: GPU Ray sensor has high performance requirements. A dedicated GPU is recommended.
3. **Ray Count**: The `ray_count` parameter significantly affects performance. Adjust according to your hardware.
4. **Environment Variables**: Ensure Gazebo environment variables are correctly set. Source `/usr/share/gazebo/setup.sh` before launching.
5. **ROS2 Plugins**: Make sure `ros-humble-gazebo-ros-pkgs` is installed.

## Performance Optimization

1. **Lower update rate**: Reduce `update_rate` from 10Hz to 5Hz
2. **Reduce ray count**: Lower `ray_count` from 24000 to 12000
3. **Use downsampling**: Set `downsample` to 2 or higher

## Contributing

Issues and Pull Requests are welcome!

If you successfully test on other Livox models, please share your configuration and results.

## License

Apache-2.0

## Acknowledgments

Based on Gazebo GpuRayPlugin, adapted for ROS2 Humble interface.

## Contact

Maintainer: Wenxi Xu  
Email: 12411711@mail.sustech.edu.cn
