# PCT Path Adapter - 全局路径适配节点

## 概述

PCT Path Adapter 是一个C++ ROS2节点，负责将PCT Planner输出的3D全局路径转换为nav2 MPPI Controller可用的2D局部路径和子目标。它处理台阶检测、速度缩放、多楼层分割等关键逻辑，使MPPI能够跟随包含高度变化的三维轨迹。

### 解决的核心问题

PCT Planner输出3D全局路径（包含楼梯、坡道等多楼层信息），而nav2 MPPI Controller仅接受2D路径。需要一个适配节点：
1. 将3D路径投影为2D供MPPI使用
2. 检测台阶区域并自动降速
3. 按楼层分割路径，支持多楼层导航
4. 选取合适的子目标引导MPPI

## 数据流架构

```
PCT Planner (Python)
    │  /pct_path (nav_msgs/Path, 3D: x,y,z)
    ▼
┌─────────────────────────────────────────────┐
│          PctPathAdapterNode (C++)            │
│                                              │
│  1. 订阅 /pct_path → 解析3D路径             │
│  2. splitByFloor() → 按台阶分割路径段        │
│  3. 定时器回调:                              │
│     - getRobotPose() → TF获取机器人位姿      │
│     - findClosestIndex() → 定位最近路径点    │
│     - 台阶前瞻检测 (step_zone_ahead米)       │
│     - 速度缩放: 台阶区0.3x, 平地1.0x        │
│     - 选取lookahead子目标                    │
│  4. 发布:                                    │
│     /plan         → 2D局部路径 (给MPPI)      │
│     /goal_pose    → 子目标位姿 (给nav2)      │
│     /step_warning → 台阶预警 (Bool)          │
│     /speed_scale  → 速度缩放因子 (Float32)   │
│     /pct_path_2d  → 完整2D投影路径           │
└─────────────────────────────────────────────┘
    │  /plan + /goal_pose
    ▼
nav2 MPPI Controller
    │  /cmd_vel
    ▼
机器人底盘
```

## 关键数据结构

### PathPoint3D - 3D路径点
```cpp
struct PathPoint3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
```

### PathSegment - 路径段
```cpp
struct PathSegment
{
  std::vector<PathPoint3D> points;
  bool has_step = false;       // 是否包含台阶
  double step_height = 0.0;    // 台阶高度
  size_t step_start_idx = 0;   // 台阶起始索引
};
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `step_height_threshold` | 0.15m | 台阶高度检测阈值 |
| `lookahead_distance` | 3.0m | 子目标前瞻距离 |
| `goal_tolerance` | 0.5m | 段终点到达判定距离 |
| `speed_scale_step` | 0.3 | 台阶区速度缩放因子 |
| `speed_scale_flat` | 1.0 | 平地区速度缩放因子 |
| `step_zone_ahead` | 2.0m | 台阶前瞻扫描距离 |
| `map_frame` | map | 全局坐标系 |
| `robot_frame` | base_link | 机器人坐标系 |

## 核心逻辑

### 台阶检测与速度调节

当路径前方 `step_zone_ahead` 范围内检测到台阶时，速度缩放因子按台阶高度线性插值：

```
ratio = min(|step_dz| / 0.5, 1.0)
scale = speed_scale_flat - ratio * (speed_scale_flat - speed_scale_step)
```

- 平地区域: `scale = 1.0`（全速）
- 低台阶(0.15m): `scale ≈ 0.8`
- 高台阶(0.5m+): `scale = 0.3`（最低速）

### 多楼层路径分割

当路径中相邻点z差超过 `step_height_threshold` 时，路径在台阶处分割为多个段。机器人完成一段后自动切换到下一段，实现多楼层导航。

### 子目标选取

从机器人最近路径点开始，沿路径前进 `lookahead_distance`，计算朝向角（yaw）写入 goal_pose 的 orientation，同时发布局部路径 `/plan`（从最近点到前方若干点）。

## 话题接口

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pct_path` | nav_msgs/Path | PCT Planner输出的3D全局路径 |
| `/cmd_vel` | geometry_msgs/Twist | 底盘速度反馈（用于判断是否卡住） |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/plan` | nav_msgs/Path | 2D局部路径（给MPPI） |
| `/goal_pose` | geometry_msgs/PoseStamped | 子目标位姿（给nav2） |
| `/speed_scale` | std_msgs/Float32 | 速度缩放因子 |
| `/step_warning` | std_msgs/Bool | 台阶预警 |
| `/pct_path_2d` | nav_msgs/Path | 完整2D投影路径 |

## 启动方式

```bash
# 单独启动
ros2 launch pct_path_adapter pct_path_adapter.launch.py

# 自定义参数启动
ros2 launch pct_path_adapter pct_path_adapter.launch.py \
    step_height_threshold:=0.2 \
    lookahead_distance:=4.0 \
    speed_scale_step:=0.2
```

## 与底盘驱动集成

`/speed_scale` 话题可在底盘驱动节点中用于缩放 `cmd_vel`：

```python
# 示例：底盘驱动节点中的速度缩放
speed_scale = 1.0

def speed_scale_callback(msg):
    global speed_scale
    speed_scale = msg.data

def cmd_vel_callback(msg):
    scaled_cmd = Twist()
    scaled_cmd.linear.x = msg.linear.x * speed_scale
    scaled_cmd.linear.y = msg.linear.y * speed_scale
    scaled_cmd.angular.z = msg.angular.z * speed_scale
    cmd_pub.publish(scaled_cmd)
```

## 编译

```bash
cd ~/git/3d_dog_navi_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select pct_path_adapter
```

## 项目结构

```
pct_path_adapter/
├── include/pct_path_adapter/
│   └── pct_path_adapter_node.hpp    # 头文件，定义数据结构和类接口
├── src/
│   └── pct_path_adapter_node.cpp    # 核心实现
├── launch/
│   └── pct_path_adapter.launch.py   # 启动文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 与 Traversability Layer 的协同

本节点与 Traversability Layer 配合构成完整的3D导航感知-规划闭环：

```
FAST-LIO2 → Traversability Layer → Costmap2D → MPPI Controller
                                        ↑
PCT Planner → PCT Path Adapter → /plan + /goal_pose
```

- **Traversability Layer**: 从点云中提取3D可通行性信息，写入nav2 costmap
- **PCT Path Adapter**: 将3D全局路径适配为2D局部路径，引导MPPI跟随三维轨迹
- **MPPI Controller**: 基于costmap进行局部避障，输出cmd_vel
