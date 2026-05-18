# 3D Dog Navigation ROS2 项目
# 建设中，目前只有手动控制

## 项目介绍

本项目是基于ROS2 Humble的3D导航仿真系统，专门为Unitree Go2W机器狗设计，集成了多种先进的路径规划算法和机器人控制技术。项目提供完整的机器狗仿真环境，支持自主导航、SLAM建图、路径规划等高级功能。

### 主要特性

- **完整的ROS2迁移**：所有组件已从ROS1迁移到ROS2 Humble
- **机器狗专用仿真**：针对Unitree Go2W机器狗优化的仿真环境
- **多算法集成**：支持PCT-planner和ego-planner路径规划算法
- **地面导航优化**：专门为机器狗地面运动优化的导航算法
- **SLAM集成**：集成FAST-LIO2激光SLAM用于实时建图
- **混合运动控制**：支持机器狗行走、小跑、奔跑等多种步态
- **RL运动控制**：集成rl_sar强化学习控制器，支持高动态运动场景
- **IMU姿态补偿**：基于IMU数据的姿态补偿，提高运动稳定性
- **上坡检测**：自动检测上坡（pitch > 15度），切换到轮腿协同前进模式
- **Gazebo仿真**：高保真的机器狗物理仿真环境

### 3D导航示例
<div align="center">
  <img src="/src/image/878355907.gif" width="800"/>
</div>

### 3D规划示例
<div align="center">
  <img src="/src/image/123141123.gif" width="800"/>
</div>

---

## 项目架构

### 核心组件

#### 机器狗仿真与控制系统
1. **unitree_go2w_ros2** - Unitree Go2W机器狗专用仿真包
   - Go2W高精度机器人模型（包含Livox Mid360激光雷达）
   - 混合运动控制器（支持行走、小跑、奔跑等步态）
   - RL强化学习控制器（基于rl_sar，支持高动态运动）
   - 状态估计器（里程计、IMU数据融合）
   - Gazebo物理仿真集成

2. **planner** - 机器狗专用路径规划器
   - 地面模式优化算法（限制Z轴运动）
   - Ego-planner轨迹优化
   - 实时避障和路径重规划
   - 支持多点路径规划

3. **FAST_LIO_ROS2_edit** - 激光SLAM系统
   - FAST-LIO2激光惯性里程计
   - 实时点云建图
   - 高精度定位与姿态估计

#### 路径规划算法
4. **pct_planner** - PCT路径规划器（多楼层全局规划）
   - 点云断层摄影环境建模（GPU加速）
   - 多楼层路径规划（楼梯、坡道、过桥）
   - A*搜索 + GPMP轨迹优化
   - 3D轨迹输出（nav_msgs/Path）
   - ROS2 Humble原生支持
   - 性能：断层图生成~40ms，路径搜索~20ms，轨迹优化~375ms

5. **ego_planner** - 快速轨迹优化算法
   - B样条轨迹优化
   - 实时避障能力
   - 支持动态环境

#### 工具与可视化
6. **rviz-3d-nav-goal-tool** - 3D导航目标工具
   - RViz插件
   - 3D目标点设置
   - 机器狗状态可视化

### 迁移状态

| 组件 | 状态 | ROS2版本 | 主要改进 |
|------|------|----------|----------|
| Go2W机器狗仿真 | ✅ 完全迁移 | ROS2 Humble | 混合运动控制器，RL控制器，状态估计器 |
| FAST-LIO2 SLAM | ✅ 完全迁移 | ROS2 Humble | 激光惯性里程计，实时建图 |
| Ego-planner | ✅ 无 | ROS2 Humble | 地面模式优化，实时避障 |
| PCT-planner | ✅ 已部署 | ROS2 Humble | 断层摄影环境建模，多楼层规划 |
| 机器狗导航系统 | ✅ 无 | ROS2 Humble | 完整自主导航流程 |
| RViz可视化工具 | ✅ 完全迁移 | ROS2 Humble | 3D导航目标插件 |

---

## 安装与配置

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **Gazebo**: Gazebo Garden 8.10.0+ (推荐) 或 Gazebo 11
- **CUDA**: 12.1 (推荐，用于rl_sar GPU推理和PCT-planner)
- **Python**: 3.10
- **Bridge包**: ros-humble-ros-gzgarden-bridge (针对Gazebo Garden)

### 依赖安装

1. **安装ROS2 Humble**
```bash
sudo apt update && sudo apt install curl software-properties-common
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-robot-localization
```

2. **安装项目依赖**
```bash
sudo apt install python3-colcon-common-extensions python3-pip liblcm-dev
sudo apt install ros-humble-topic-tools
sudo apt install ros-humble-ros-gzgarden-bridge
pip3 install numpy scipy pybind11
```

3. **配置CUDA和LibTorch**（如需要）
```bash
# 安装CUDA Toolkit 12.1（WSL2环境）
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-1

# 添加环境变量到 ~/.bashrc
echo 'export PATH=/usr/local/cuda-12.1/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# 验证CUDA安装
nvcc --version

# 下载LibTorch GPU版本（cu121）到rl_sar
cd ~/git/3d_dog_navi_ros2/src/rl_sar/library/inference_runtime
rm -rf libtorch  # 删除旧的CPU版本（如果有）
wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcu121.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcu121.zip
rm libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcu121.zip
```

### 项目编译

 **编译ROS2包**
```bash
cd ~/git/3d_dog_navi_ros2
source /opt/ros/humble/setup.bash
```

# 编译特定包：
```bash
# 编译机器狗核心包（推荐）
colcon build --symlink-install --packages-select go2w_config go2w_control go2w_description champ champ_base champ_bringup champ_config champ_description champ_gazebo champ_msgs champ_navigation

# 编译RL控制器包（需要先编译依赖）
colcon build --symlink-install --packages-select robot_msgs
colcon build --symlink-install --packages-select robot_joint_controller
colcon build --symlink-install --packages-select rl_sar

# 编译SLAM和导航包
colcon build --symlink-install --packages-select FAST_LIO_ROS2_edit ego_planner planner

# 编译完整机器狗导航系统
colcon build --symlink-install --packages-select go2w_config go2w_control go2w_description champ FAST_LIO_ROS2_edit ego_planner planner

# 完整编译所有包（推荐用于开发）
colcon build --symlink-install
```

> **注意**：rl_sar 包编译前需先下载推理运行时（LibTorch），执行：
> ```bash
> cd ~/git/3d_dog_navi_ros2/src/rl_sar
> bash scripts/download_inference_runtime.sh libtorch
> ```

4. **配置环境变量**
```bash
echo "source ~/3d_dog_navi_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 使用指南

### 快速开始 - 机器狗自主导航

#### 方法一：一键启动完整系统
```bash
# 启动完整的机器狗自主导航系统
ros2 launch ego_planner dog_ego_planner_integrated.launch.py
```

此命令将自动启动：
- Gazebo仿真环境
- Go2W机器狗模型和控制器
- FAST-LIO2激光SLAM
- Ego-planner路径规划器
- 轨迹服务器
- RViz可视化

#### 方法二：分步启动（推荐用于调试）

**终端1 - 启动机器狗仿真环境**
```bash
ros2 launch go2w_control hybrid_controller.launch.py
ros2 launch go2w_control hybrid_controller_ignition.launch.py
```

**终端2 - 启动SLAM和导航系统**
```bash
ros2 launch ego_planner advanced_param.launch.py
```

**终端3 - 启动键盘控制（可选）**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 方法三：RL强化学习控制器启动

RL控制器基于[rl_sar](https://github.com/ypat999/rl_sar)（已适配Ignition Gazebo），提供高动态运动能力，适合需要快速移动和敏捷转向的场景。

```bash
# 启动RL控制器仿真环境
ros2 launch go2w_control rl_controller_ignition.launch.py
```

此命令将自动启动：
- Ignition Gazebo仿真环境
- Go2W机器狗模型（RL控制器配置）
- ros2_control控制器管理器 + robot_joint_controller
- rl_sim RL推理节点（加载Go2W policy）
- Ignition-ROS2桥接（IMU、关节状态等）
- 关节状态广播器

**RL控制器操作方式**：
- **键盘**：`W/A/S/D` 前后左右，`Q/E` 偏航转向
- **手柄**：左摇杆移动，右摇杆偏航
- **命令速度**：发布 `geometry_msgs/Twist` 到 `/cmd_vel`

**RL控制器FSM状态机**：
| 状态 | 说明 | 触发条件 |
|------|------|----------|
| Passive | 关节自由，无控制 | 启动初始状态 |
| GetUp | 从趴下姿态站起 | 按键/手柄触发 |
| RLLocomotion | RL策略控制行走 | 站起后自动进入 |
| GetDown | 从站立姿态趴下 | 按键/手柄触发 |

**更换RL策略模型**：
```bash
# 替换policy文件即可，位于：
~/git/3d_dog_navi_ros2/src/rl_sar/policy/go2w/robot_lab/policy.pt

# 如需更换策略配置，修改：
~/git/3d_dog_navi_ros2/src/rl_sar/policy/go2w/robot_lab/config.yaml
```

> **注意**：RL控制器与混合运动控制器使用不同的SDF模型和ros2_control配置，不可同时启动。

#### 机器狗控制操作

**键盘控制**（teleop_twist_keyboard）：
- **W/S**: 前进/后退
- **A/D**: 左转/右转  
- **Q/E**: 左移/右移
- **R/F**: 上升/下降（地面模式下限制）

**自主导航**：
- 在RViz中使用3D导航目标工具设置目标点
- 机器狗将自动规划路径并导航到目标位置

#### 可视化监控
```bash
# 启动RViz可视化
ros2 launch rviz-3d-nav-goal-tool navigation.rviz.launch.py

# 查看话题列表
ros2 topic list

# 监控机器狗状态
ros2 topic echo /odom
ros2 topic echo /livox/lidar
```

### 模式切换配置

项目支持地面和空中两种导航模式，可通过启动参数切换：

```bash
# 地面模式（默认）
ros2 launch planner navigation.launch.py mode:=ground

# 空中模式
ros2 launch planner navigation.launch.py mode:=aerial
```

### 机器狗专用配置

#### 传感器话题配置
机器狗仿真系统使用以下标准话题：
- **激光雷达**: `/livox/lidar` (PointCloud2类型)
- **IMU数据**: `/livox/imu` 
- **里程计**: `/odom` (来自状态估计器)
- **控制命令**: `/cmd_vel` (Twist消息)

#### 导航系统配置
机器狗导航系统使用以下话题映射：
- **SLAM输入**: `/livox/lidar` + `/livox/imu` → FAST-LIO2
- **规划器输入**: `/odom` + `/livox/lidar` → Ego-planner
- **控制输出**: `[drone_id]_plan_vis/goal_point` → 轨迹服务器

#### 地面模式参数
机器狗使用地面模式优化参数：
```yaml
enable_ground_mode: True
xy_extend: 5
z_extend: 1
z_penalty_weight: 1.2
xy_gradient_weight: 1.0
```

#### 混合控制器参数
机器狗混合运动控制器支持以下关键参数：
```yaml
# 姿态补偿参数
enable_pose_compensation: true      # 启用IMU姿态补偿
compensation_gain: 0.2              # 姿态补偿增益（0.0-1.0）
max_compensation_height: 0.1        # 最大补偿高度（米）

# 上坡检测参数
pitch_threshold: 15.0               # 上坡检测阈值（度）
```

**参数说明**：
- `enable_pose_compensation`: 是否启用基于IMU的姿态补偿功能
- `compensation_gain`: 姿态补偿的增益系数，值越大补偿越强
- `max_compensation_height`: 姿态补偿的最大高度限制
- `pitch_threshold`: 触发上坡模式的pitch角度阈值

**启动时设置参数**：
```bash
# 使用默认参数启动
ros2 launch go2w_control hybrid_controller_ignition.launch.py

# 自定义参数启动
ros2 launch go2w_control hybrid_controller_ignition.launch.py compensation_gain:=0.3 max_compensation_height:=0.15
```

### 控制器操作

#### Go2W机器狗控制
使用键盘控制（teleop_twist_keyboard）：
- **W/S**: 前进/后退
- **A/D**: 左转/右转
- **Q/E**: 左移/右移
- **R/F**: 上升/下降（地面模式下限制）

#### 步态切换（高级功能）
机器狗支持多种步态模式：
- **行走模式**：低速稳定移动
- **小跑模式**：中等速度平衡移动
- **奔跑模式**：高速移动（需要足够空间）

---

## 高级功能

### 断层摄影环境建模

PCT规划器集成了先进的断层摄影技术，用于环境的三维建模：

```bash
# 生成环境断层图
cd ~/git/3d_dog_navi_ros2/src/pct_planner/tomography/scripts
python3 run_standalone.py --scene Building

# 路径规划
cd ~/git/3d_dog_navi_ros2/src/pct_planner/planner/scripts
python3 plan_standalone.py --tomo building2_9 --start -5 -3 --end 5 3
```

**PCT Planner 环境变量设置**（添加到 ~/.bashrc）：
```bash
export LD_LIBRARY_PATH=~/git/3d_dog_navi_ros2/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:~/git/3d_dog_navi_ros2/src/pct_planner/planner/lib/build/src/common/smoothing:$LD_LIBRARY_PATH
export PYTHONPATH=~/git/3d_dog_navi_ros2/src/pct_planner/planner/lib:$PYTHONPATH
```

**PCT Planner 与机器狗系统集成**：
```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   PCT Planner   │     │  EGO Planner    │     │   机器狗控制    │
│   (全局路径)    │────▶│   (局部优化)    │────▶│   (执行)        │
│   多楼层支持    │     │   3D避障        │     │   /cmd_vel      │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        │                       │
        ▼                       ▼
   /pct_path              /position_cmd
   (nav_msgs/Path)        → 转换为/cmd_vel
```

**PCT Planner 性能指标**：
| 指标 | 数值 |
|------|------|
| 断层图生成 | ~40ms (85万点) |
| 路径搜索 | ~20ms |
| 轨迹优化 | ~375ms |
| 输出航点 | 1000+ |

### 自定义场景配置

项目支持多种预定义场景，也可自定义场景配置：

```yaml
# 在pct_planner_ros2/config/目录下创建自定义场景
scene_config:
  pcd:
    file_name: "custom_scene.pcd"
  map:
    resolution: 0.1
    ground_h: 0.0
    slice_dh: 0.5
```

### 性能优化

对于高性能需求，可启用C++加速：

```bash
# 编译C++组件
cd ~/3d_dog_navi_ros2_ws/src/pct_planner_ros2
./build_cpp.sh

# 使用C++加速版本
ros2 run pct_planner_ros2 plan_node --ros-args -p use_cpp:=true
```

---

## 故障排除与调试

### 机器狗专用故障排除

#### 1. **机器狗无法站立或移动**
- **问题**：机器狗在Gazebo中无法站立或响应控制命令
- **解决方案**：
  ```bash
  # 检查控制器状态
  ros2 node list | grep controller
  
  # 重启控制器
  ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: ['joint_group_effort_controller'], stop_controllers: [], strictness: 1}"
  ```

#### 2. **SLAM定位漂移或丢失**
- **问题**：FAST-LIO2定位不准确或丢失
- **解决方案**：
  ```bash
  # 检查传感器数据
  ros2 topic echo /livox/lidar --no-arr | head -5
  ros2 topic echo /livox/imu --no-arr | head -5
  
  # 重启SLAM节点
  ros2 lifecycle set /fast_lio2_dog_odom configure
  ros2 lifecycle set /fast_lio2_dog_odom activate
  ```

#### 3. **路径规划失败**
- **问题**：Ego-planner无法找到可行路径
- **解决方案**：
  ```bash
  # 检查地图数据
  ros2 topic echo /grid_map/occupancy_inflate --no-arr | head -3
  
  # 调整规划参数
  ros2 param set /drone_0_ego_planner_node grid_map/obstacles_inflation 0.15
  ```

### 通用故障排除

#### 1. **Gazebo无法启动**
   ```bash
   # 检查Gazebo安装
   gazebo --version
   
   # 重置Gazebo模型数据库
   rm -rf ~/.gazebo/
   ```

#### 2. **ROS2节点无法通信**
   ```bash
   # 检查ROS2环境
   echo $ROS_DISTRO
   
   # 重新source环境
   source /opt/ros/humble/setup.bash
   source ~/3d_dog_navi_ros2/install/setup.bash
   ```

### 调试工具

使用ROS2内置工具进行调试：

```bash
# 查看节点状态
ros2 node list

# 查看话题列表
ros2 topic list

# 监控机器狗关键话题
ros2 topic echo /odom                    # 里程计数据
ros2 topic echo /livox/lidar --no-arr    # 激光雷达数据（简化输出）
ros2 topic echo /cmd_vel                 # 控制命令
ros2 topic echo /planning/bspline        # 规划轨迹

# 查看节点图
rqt_graph
```

### 已知问题与解决方案

1. **包名验证错误："'package.xml' is not a valid package name"**
   - **问题**：启动时出现包名验证错误
   - **原因**：包索引目录中存在错误的文件名
   - **解决方案**：
     ```bash
     # 检查并修复包索引文件
     cd ~/3d_dog_navi_ros2/install
     find . -name "package.xml" -path "*/ament_index/resource_index/packages/*"
     
     # 如果发现错误的文件，重命名为正确的包名
     mv pct_planner_ros2/share/ament_index/resource_index/packages/package.xml \
        pct_planner_ros2/share/ament_index/resource_index/packages/pct_planner_ros2
     ```

2. **FindPackageShare对象类型错误**
   - **问题**：启动时出现"expected str, bytes or os.PathLike object, not FindPackageShare"错误
   - **原因**：FindPackageShare对象被错误地传递给os.path.join()函数
   - **解决方案**：使用PathJoinSubstitution替代os.path.join()

3. **机器狗话题映射错误**
   - **问题**：节点间话题通信失败
   - **原因**：话题名称不匹配
   - **解决方案**：检查并更新启动文件中的话题映射配置

4. **Gazebo Harmonic插件加载失败**
   - **问题**：gz_ros2_control插件无法加载，提示"library does not contain requested plugin"
   - **原因**：插件编译版本与Gazebo版本不匹配
   - **解决方案**：
     ```bash
     # 重新编译gz_ros2_control插件以支持Gazebo Garden
     export GZ_VERSION=garden
     colcon build --symlink-install --packages-select gz_ros2_control
     ```

5. **机器狗轮子在Gazebo中不可见**
   - **问题**：Gazebo中看不到机器狗轮子，但RViz中可见
   - **原因**：SDF文件中脚的视觉模型使用了错误的mesh文件
   - **解决方案**：检查并修正go2w.sdf中的视觉模型配置，确保使用正确的轮子模型（left_wheel.dae/right_wheel.dae）

6. **XTDrone2传感器错误**
   - **问题**：PX4仿真中出现"Compass Sensor 0 missing"或"ekf2 missing data"错误
   - **原因**：Gazebo模型中缺少磁力计传感器配置
   - **解决方案**：
     ```bash
     # 在PX4 Gazebo模型中添加磁力计传感器
     # 启用PX4磁力计支持：param set-default EKF2_MAG_TYPE 3
     # 添加ROS-Gazebo磁力计数据桥接
     ```

7. **topic_tools包缺失**
   - **问题**：启动时出现"package 'topic_tools' not found"错误
   - **原因**：缺少ROS2 topic_tools包
   - **解决方案**：
     ```bash
     sudo apt install ros-humble-topic-tools
     ```

8. **IMU数据无法转发**
   - **问题**：ROS2中`/imu/data`话题无数据，但Gazebo中有数据
   - **原因**：ros_gz_bridge版本与Gazebo版本不匹配
   - **解决方案**：
     ```bash
     # 检查Gazebo版本
     gz sim --versions
     
     # 安装正确的bridge包
     # 对于Gazebo Garden (7.x)
     sudo apt install ros-humble-ros-gzgarden-bridge
     
     # 对于Gazebo Harmonic (8.x)
     sudo apt install ros-humble-ros-gzharmonic-bridge
     
     # 确认launch文件使用正确的包名
     # Gazebo Garden: package='ros_gzgarden_bridge'
     # Gazebo Harmonic: package='ros_gzharmonic_bridge'
     ```

9. **Bridge版本不匹配**
   - **问题**：bridge无法连接Gazebo，提示库版本错误
   - **原因**：使用Ignition版本的bridge连接Gazebo Garden
   - **解决方案**：
     ```bash
     # 卸载旧版本
     sudo apt remove ros-humble-ros-gz-bridge
     
     # 安装Gazebo Garden专用bridge
     sudo apt install ros-humble-ros-gzgarden-bridge
     ```

---

## 开发指南

### 项目结构

```
3d_dog_navi_ros2/
├── src/
│   ├── pct_planner/              # PCT规划器（多楼层全局规划）
│   ├── rl_sar/                   # RL强化学习控制器（submodule）
│   ├── planner/                   # EGO路径规划器
│   ├── unitree_go2w_ros2/         # Unitree Go2W机器人仿真
│   ├── unitree_ros2_sim/          # Unitree机器人仿真（兼容Go1）
│   ├── uav_simulator/             # 无人机仿真
│   └── rviz-3d-nav-goal-tool/     # RViz插件
├── launch/                        # 启动文件
├── config/                        # 配置文件
└── scripts/                       # 工具脚本
```

### 扩展开发

1. **添加新的规划算法**
   - 在`planner`包中实现新的规划器
   - 遵循ROS2节点接口规范
   - 添加对应的启动文件和配置

2. **自定义机器人模型**
   - 在`unitree_ros2_sim`中添加新模型
   - 更新URDF文件和控制器配置
   - 测试Gazebo集成

3. **开发新的传感器插件**
   - 创建ROS2传感器驱动
   - 集成到仿真环境中
   - 提供标准的话题接口

### 贡献指南

1. Fork项目仓库
2. 创建特性分支
3. 提交更改
4. 推送到分支
5. 创建Pull Request

---

## 许可证

本项目基于MIT许可证开源。详见[LICENSE](LICENSE)文件。

## 致谢

- [PCT-planner](https://github.com/ypat999/PCT_planner.git) - 点云断层摄影路径规划（多楼层导航）
- [rl_sar](https://github.com/ypat999/rl_sar) - 强化学习机器人控制器（Go2W RL运动控制）
- [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) - 快速轨迹优化算法
- [Unitree Robotics](https://www.unitree.com/) - 机器人硬件和仿真模型
- [ROS2社区](https://docs.ros.org/) - 机器人操作系统框架

---

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目Issues: [GitHub Issues]
- 邮箱: [项目维护者邮箱]
- 文档: [项目Wiki]

## 更新日志

### v2.4.0 (2025-05-16)
- **RL强化学习控制器集成**
  - 添加rl_sar作为git submodule（fork自ypat999/rl_sar）
  - 适配Ignition Gazebo环境（Gazebo Classic依赖改为可选）
  - 创建RL专用ros2_control配置（rl_joint_controller.yaml）
  - 创建RL专用SDF模型（go2w_rl.sdf）
  - 创建RL启动文件（rl_controller_ignition.launch.py）
  - 下载LibTorch推理运行时，支持policy.pt加载
  - Go2W预训练策略：robot_lab（16自由度，含4轮关节）

### v2.3.0 (2025-04-20)
- **PCT Planner 部署完成**
  - 编译GTSAM 4.1.1和OSQP第三方库
  - 编译Python绑定库（a_star, ele_planner, traj_opt）
  - 修复CuPy版本匹配问题（CUDA 11.x）
  - 测试通过：断层图生成~40ms，路径规划~20ms，轨迹优化~375ms
  - 添加完整的中文README文档
  - 支持多楼层路径规划（楼梯、坡道、过桥）

### v2.2.0 (2025-02-26)
- **混合运动控制器增强**
  - 添加IMU订阅和pitch角度检测
  - 实现上坡自动检测（pitch > 15度）
  - 上坡时自动切换到轮腿协同前进模式
  - 平地时轮子控制x方向，腿部控制y和z方向
- **姿态补偿功能完善**
  - 添加enable_pose_compensation参数支持
  - 添加compensation_gain参数（默认0.2）
  - 添加max_compensation_height参数（默认0.1）
  - 参数从launch文件传递到quadruped_controller节点
- **Bridge版本兼容性修复**
  - 修复ros_gz_bridge与Gazebo Garden版本不匹配问题
  - 安装ros-humble-ros-gzgarden-bridge包
  - 更新launch文件使用正确的bridge包名
  - 修复IMU数据转发问题

### v2.1.0 (2025-02-25)
- **Gazebo Harmonic兼容性修复**
  - 重新编译gz_ros2_control插件以支持Gazebo Harmonic 8.10.0
  - 修正SDF插件名称从gz_ros2_control::GazeboSystemPlugin到gz_ros2_control::GazeboSimROS2ControlPlugin
  - 修复XML插件描述文件安装问题
- **机器狗轮子显示修复**
  - 修正go2w.sdf中脚的视觉模型从foot.dae到对应的轮子模型（left_wheel.dae/right_wheel.dae）
  - 解决Gazebo中看不到轮子但RViz中可见的问题
- **依赖包完善**
  - 添加ros-humble-topic-tools包依赖
  - 验证并修复YAML配置文件语法
- **XTDrone2传感器问题修复**
  - 在PX4 Gazebo模型中添加磁力计传感器配置
  - 启用PX4磁力计支持（EKF2_MAG_TYPE设置为3）
  - 添加ROS-Gazebo磁力计数据桥接

### v2.0.0 (2024-01-01)
- 完整迁移到ROS2 Humble
- 新增地面/空中模式切换
- 集成PCT规划器断层摄影模块
- 优化仿真性能和稳定性

### v1.0.0 (2023-12-31)
- 初始版本发布
- 基于ROS1的3D导航系统
- 基础路径规划功能