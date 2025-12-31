# Livox GPU 激光雷达仿真 ROS2 插件

> **注意**: 目前仅在 **Mid-360** 上完整测试过。其他型号（Avia, HAP, Horizon, Mid-40, Mid-70, Tele）的配置文件已包含，但未经过充分测试。

[English Documentation](README.md)

## 功能特性

- 基于 Gazebo GPU Ray 传感器的高性能点云仿真
- 发布 `sensor_msgs/PointCloud2` 消息
- 支持多种 Livox 激光雷达型号的扫描模式
- 支持自定义话题名称、坐标系和命名空间
- 包含预配置的 URDF/Xacro 文件和 3D 模型

## 依赖项

### 系统要求
- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Gazebo 11
- SDFormat 9

### ROS2 包依赖
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

### 编译依赖
- OpenMP
- gazebo_dev
- pkg-config

## 安装与构建

### 1. 克隆仓库

```bash
cd ~/your_workspace/src
git clone git@gihub.com:ttwards/livox_gazebo_ros2_gpu_simulation livox_sim_gpu_ros2
```

### 2. 安装依赖

```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

```bash
cd ~/your_workspace
colcon build --packages-select livox_gazebo_ros2_gpu_simulation
```

### 4. 设置环境

```bash
source install/setup.bash
```

编译完成后，环境 hook 会自动设置 `GAZEBO_PLUGIN_PATH`，将插件路径添加到 Gazebo 的搜索路径中。

## 使用方法

### 使用预配置的 Xacro 文件

项目已包含预配置的 Xacro 文件，可以直接在你的机器人 URDF 中引用：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- 包含 Mid-360 激光雷达 -->
  <xacro:include filename="$(find livox_gazebo_ros2_gpu_simulation)/urdf/mid360.xacro"/>
  
  <!-- 将激光雷达附加到机器人的某个 link 上 -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="mid360_base"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  
</robot>
```

### 可用的 Xacro 文件

- `urdf/mid360.xacro` - Mid-360（已测试）
- `urdf/avia.xacro` - Avia
- `urdf/horizon.xacro` - Horizon
- `urdf/mid40.xacro` - Mid-40
- `urdf/mid70.xacro` - Mid-70
- `urdf/HAP.xacro` - HAP
- `urdf/tele.xacro` - Tele-15

### 插件参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ray_count` | int | 24000 | 每帧射线数量 |
| `min_range` | double | 0.1 | 最小测距范围（米） |
| `max_range` | double | 200.0 | 最大测距范围（米） |
| `topic` | string | "pointcloud" | 发布的 ROS2 话题名称 |
| `frame_id` | string | "lidar_link" | TF 坐标系名称 |
| `update_rate` | double | 10.0 | 更新频率（Hz） |
| `downsample` | int | 1 | 降采样因子（1=无降采样） |
| `csv_file_name` | string | "mid360" | 扫描模式 CSV 文件名（不含扩展名） |

### 查看数据

```bash
# 查看点云话题
ros2 topic list | grep pointcloud

# 查看点云数据
ros2 topic echo /pointcloud

# 在 RViz2 中可视化
rviz2
# 添加 PointCloud2 显示，订阅 /pointcloud 话题
```

## 文件结构

```
livox_sim_gpu_ros2/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS2 包描述文件
├── README.md                   # 英文文档
├── README_zh.md                # 本文件（中文）
├── livox_sim_gpu_laser.xml     # 插件描述
├── include/
│   └── livox_sim_plugins/
│       └── livox_sim_gpu_laser.h    # 插件头文件
├── src/
│   ├── livox_sim_gpu_laser.cpp      # 插件实现
│   └── laser_listener.cpp           # 测试监听节点
├── launch/
│   └── test_gpu_laser.launch.py     # 测试启动文件
├── urdf/                       # URDF/Xacro 模型文件
│   ├── mid360.xacro
│   ├── avia.xacro
│   ├── horizon.xacro
│   ├── mid40.xacro
│   ├── mid70.xacro
│   ├── HAP.xacro
│   └── tele.xacro
├── scan_mode/                  # 扫描模式 CSV 文件
│   ├── mid360.csv
│   ├── avia.csv
│   ├── horizon.csv
│   ├── mid40.csv
│   ├── mid70.csv
│   ├── HAP.csv
│   └── tele.csv
├── meshes/
│   └── mid-360-scaled.dae      # 3D 模型
└── env-hooks/
    └── livox_sim_gpu_ros2.sh.in # 环境设置 hook
```

## 注意事项

1. **GPU 支持**: 此插件需要 GPU 支持。如果你的系统没有 GPU 或 Gazebo 无法初始化渲染引擎，传感器将无法工作。
2. **性能**: GPU Ray 传感器对系统性能要求较高，建议使用独立显卡。
3. **射线数量**: `ray_count` 参数会显著影响性能，根据你的硬件调整。
4. **环境变量**: 确保 Gazebo 环境变量正确设置。启动前请 source `/usr/share/gazebo/setup.sh`。
5. **ROS2 插件**: 确保安装了 `ros-humble-gazebo-ros-pkgs`。

## 性能优化建议

1. **降低更新频率**: 将 `update_rate` 从 10Hz 降低到 5Hz
2. **减少射线数**: 将 `ray_count` 从 24000 降低到 12000
3. **使用降采样**: 设置 `downsample` 为 2 或更高
## 开发与贡献

欢迎提交 Issues 和 Pull Requests！

如果你在其他 Livox 型号上测试成功，请分享你的配置和结果。

## 许可证

Apache-2.0

## 致谢

基于 Gazebo GpuRayPlugin 开发，适配 ROS2 Humble 接口。

## 联系方式

维护者：Wenxi Xu  
邮箱：12411711@mail.sustech.edu.cn

