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
4. **pct_planner_ros2** - PCT路径规划器
   - 点云断层摄影环境建模
   - 支持C++/Python混合编程
   - 完整的ROS2节点实现

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
| Go2W机器狗仿真 | ✅ 完全迁移 | ROS2 Humble | 混合运动控制器，状态估计器 |
| FAST-LIO2 SLAM | ✅ 完全迁移 | ROS2 Humble | 激光惯性里程计，实时建图 |
| Ego-planner | ✅ 完全迁移 | ROS2 Humble | 地面模式优化，实时避障 |
| PCT-planner | ✅ 完全迁移 | ROS2 Humble | 断层摄影环境建模 |
| 机器狗导航系统 | ✅ 完全迁移 | ROS2 Humble | 完整自主导航流程 |
| RViz可视化工具 | ✅ 完全迁移 | ROS2 Humble | 3D导航目标插件 |

---

## 安装与配置

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **CUDA**: 11.0+ (推荐，用于PCT-planner和控制器)
- **Python**: 3.8+
- **Gazebo**: Gazebo 11+

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
pip3 install numpy scipy pybind11
```

3. **配置CUDA和libtorch**（如需要）
```bash
# 安装CUDA工具包
sudo apt install nvidia-cuda-toolkit

# 下载libtorch
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip
unzip libtorch*.zip -d /opt/
```

### 项目编译

 **编译ROS2包**
```bash
cd ~/3d_dog_navi_ros2
source /opt/ros/humble/setup.bash
```

# 编译特定包：
```bash
# 编译机器狗核心包（推荐）
colcon build --symlink-install --packages-select go2w_config go2w_control go2w_description champ champ_base champ_bringup champ_config champ_description champ_gazebo champ_msgs champ_navigation

# 编译SLAM和导航包
colcon build --symlink-install --packages-select FAST_LIO_ROS2_edit ego_planner planner

# 编译完整机器狗导航系统
colcon build --symlink-install --packages-select go2w_config go2w_control go2w_description champ FAST_LIO_ROS2_edit ego_planner planner

# 完整编译所有包（推荐用于开发）
colcon build --symlink-install
```

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
```

**终端2 - 启动SLAM和导航系统**
```bash
ros2 launch ego_planner advanced_param.launch.py
```

**终端3 - 启动键盘控制（可选）**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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
ros2 run pct_planner_ros2 tomography_node --ros-args -p scene:=Spiral

# 查看生成的层数据
ros2 topic list | grep layer
```

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
   - **问题**：编译planner_manager.cpp时出现私有成员访问错误
   - **原因**：直接访问BsplineOptimizer类的私有成员enable_ground_mode_
   - **解决方案**：通过节点参数读取参数值，而不是直接访问私有成员查看节点图
rqt_graph
```

---

## 开发指南

### 项目结构

```
3d_dog_navi_ros2/
├── src/
│   ├── pct_planner_ros2/          # PCT规划器ROS2版本
│   ├── planner/                   # 路径规划器
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

- [PCT-planner](https://github.com/byangw/PCT_planner.git) - 点云断层摄影路径规划
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

### v2.0.0 (2024-01-01)
- 完整迁移到ROS2 Humble
- 新增地面/空中模式切换
- 集成PCT规划器断层摄影模块
- 优化仿真性能和稳定性

### v1.0.0 (2023-12-31)
- 初始版本发布
- 基于ROS1的3D导航系统
- 基础路径规划功能