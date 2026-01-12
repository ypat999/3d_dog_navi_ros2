# 3D Dog Navigation ROS2 项目

## 项目介绍

本项目是基于ROS2 Humble的3D导航仿真系统，集成了多种先进的路径规划算法和机器人控制技术。项目支持Unitree Go2W机器人在Gazebo仿真环境中的3D导航任务，并提供了完整的ROS2生态系统支持。

### 主要特性

- **完整的ROS2迁移**：所有组件已从ROS1迁移到ROS2 Humble
- **多算法集成**：支持PCT-planner和ego-planner路径规划算法
- **地面/空中模式切换**：支持机器人地面导航和空中无人机导航模式
- **断层摄影环境感知**：集成点云断层摄影技术用于环境建模
- **强化学习控制**：集成Unitree机器人的强化学习控制器
- **Gazebo仿真**：完整的机器人仿真环境

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

1. **pct_planner_ros2** - PCT路径规划器的完整ROS2迁移版本
   - 包含断层摄影模块(tomography)
   - 支持C++/Python混合编程
   - 完整的ROS2节点实现

2. **planner** - 路径规划器（支持地面/空中模式切换）
   - A*搜索算法
   - B样条轨迹优化
   - 可配置的地面/空中导航模式

3. **unitree_go2w_ros2** - Unitree Go2W机器人ROS2仿真包
   - Go2W机器人模型
   - 强化学习控制器
   - Gazebo集成

4. **unitree_ros2_sim** - Unitree机器人ROS2仿真包（兼容Go1）
   - Go1机器人模型
   - 控制器支持
   - Gazebo集成

5. **uav_simulator** - 无人机仿真环境
   - 支持无人机导航
   - 与地面机器人共享规划算法

6. **rviz-3d-nav-goal-tool** - 3D导航目标工具
   - RViz插件
   - 3D目标点设置

### 迁移状态

| 组件 | 状态 | ROS2版本 | 主要改进 |
|------|------|----------|----------|
| PCT-planner | ✅ 完全迁移 | ROS2 Humble | 完整ROS2节点，断层摄影模块 |
| Planner | ✅ 完全迁移 | ROS2 Humble | 地面/空中模式切换 |
| Unitree仿真 | ✅ 完全迁移 | ROS2 Humble | 强化学习控制器 |
| UAV仿真 | ✅ 完全迁移 | ROS2 Humble | 无人机导航支持 |
| RViz工具 | ✅ 完全迁移 | ROS2 Humble | 3D导航目标插件 |

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
# 编译Unitree Go2W相关包（推荐使用symlink编译）
colcon build --symlink-install --packages-select go2w_config go2w_control go2w_description champ champ_base champ_bringup champ_config champ_description champ_gazebo champ_msgs champ_navigation

# 编译Unitree Go1相关包（兼容）
colcon build --symlink-install --packages-select go1_gazebo go1_description go1_navigation ros2_unitree_legged_msgs ros2_unitree_legged_control unitree_guide2

# 编译规划器包
colcon build --symlink-install --packages-select planner pct_planner_ros2

# 编译其他包
colcon build --symlink-install --packages-select uav_simulator rviz-3d-nav-goal-tool

# 完整编译所有包（推荐）
colcon build --symlink-install
```

4. **配置环境变量**
```bash
echo "source ~/3d_dog_navi_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 使用指南

### 快速开始

#### 1. 启动Gazebo仿真环境
```bash
# 终端1 - 启动完整的Go2W机器人仿真（包括Gazebo、控制器和混合运动控制器）
ros2 launch go2w_control hybrid_controller.launch.py

# 终端2 - 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 2. 启动路径规划器

**地面机器人模式**:
```bash
ros2 launch planner ground_navigation.launch.py mode:=ground
```

**空中无人机模式**:
```bash
ros2 launch planner aerial_navigation.launch.py mode:=aerial
```

#### 3. 启动PCT规划器
```bash
# 启动断层摄影模块
ros2 run pct_planner_ros2 tomography_node --ros-args -p scene:=Building

# 启动PCT规划器
ros2 run pct_planner_ros2 plan_node --ros-args -p scene:=Building
```

#### 4. RViz可视化
```bash
ros2 launch rviz-3d-nav-goal-tool navigation.rviz.launch.py
```

### 模式切换配置

项目支持地面和空中两种导航模式，可通过启动参数切换：

```bash
# 地面模式（默认）
ros2 launch planner navigation.launch.py mode:=ground

# 空中模式
ros2 launch planner navigation.launch.py mode:=aerial
```

### 控制器操作

#### Go2W机器人控制
使用键盘控制（teleop_twist_keyboard）：
- **W/S**: 前进/后退
- **A/D**: 左转/右转
- **Q/E**: 左移/右移
- **R/F**: 上升/下降

#### Go1机器人控制（兼容模式）
在控制器运行后：
- **按键 2**: 机器人站立
- **按键 6**: 切换为RL模式（接收`cmd_vel`消息）
- **再次按键 2**: 重新启动控制器

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

## 故障排除

### 常见问题

1. **Gazebo无法启动**
   ```bash
   # 检查Gazebo安装
   gazebo --version
   
   # 重置Gazebo模型数据库
   rm -rf ~/.gazebo/
   ```

2. **ROS2节点无法通信**
   ```bash
   # 检查ROS2环境
   echo $ROS_DISTRO
   
   # 重新source环境
   source /opt/ros/humble/setup.bash
   source ~/3d_dog_navi_ros2_ws/install/setup.bash
   ```

3. **PCT规划器依赖问题**
   ```bash
   # 检查Python依赖
   pip3 list | grep numpy
   
   # 重新安装依赖
   pip3 install -r src/pct_planner_ros2/requirements.txt
   ### 调试工具

使用ROS2内置工具进行调试：

```bash
# 查看节点状态
ros2 node list

# 查看话题列表
ros2 topic list

# 监控特定话题
ros2 topic echo /cmd_vel
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

3. **编译错误：私有成员访问问题**
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