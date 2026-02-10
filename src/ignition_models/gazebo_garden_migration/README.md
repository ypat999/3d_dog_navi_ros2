# Gazebo Garden 迁移项目

本项目包含从 Gazebo 11 (Classic) 迁移到 Gazebo Garden 的转换文件和配置。

## 项目结构

```
gazebo_garden_migration/
├── worlds/                 # 转换后的世界文件
│   └── Building.world      # 建筑环境世界文件 (SDFormat 1.9)
├── models/                 # 模型文件
│   └── go2w/              # 机器狗模型
│       ├── model.config   # 模型配置文件
│       └── go2w.sdf       # 机器狗SDF模型 (SDFormat 1.9)
├── launch/                 # 启动文件
│   └── gazebo_garden.launch.py  # Gazebo Garden启动文件
├── config/                 # 配置文件
│   ├── bridge.yaml         # Gazebo-ROS桥接配置
│   └── rviz_config.rviz   # RViz可视化配置
└── test_gazebo_garden.py  # 测试脚本
```

## 主要变更

### 1. SDFormat 版本升级
- **原版本**: SDFormat 1.5
- **新版本**: SDFormat 1.9
- **变更**: 更新所有SDF文件的版本声明

### 2. 物理引擎配置
- **原配置**: ODE物理引擎
- **新配置**: DART物理引擎 (Gazebo Garden推荐)
- **变更**: 简化物理配置，使用Gazebo Garden默认设置

### 3. 模型结构优化
- 简化机器狗模型，移除复杂的几何细节
- 使用基本几何体替代复杂网格
- 优化关节限制和物理参数

### 4. 插件系统
- 使用Gazebo Garden的新插件架构
- 移除旧的Gazebo Classic插件
- 配置新的系统插件

## 使用方法

### 1. 环境要求
- Ubuntu 22.04+
- ROS 2 Humble
- Gazebo Garden (最新版本)
- ros_gz_sim 和 ros_gz_bridge 包

### 2. 安装依赖
```bash
sudo apt update
sudo apt install ros-humble-gz-sim ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

### 3. 测试迁移
```bash
cd /home/ywj/gazebo_garden_migration
python3 test_gazebo_garden.py
```

### 4. 启动仿真
```bash
# 在新的终端中
cd /home/ywj/gazebo_garden_migration
ros2 launch launch/gazebo_garden.launch.py
```

## 文件说明

### Building.world
- 转换后的建筑环境世界
- 使用SDFormat 1.9格式
- 配置DART物理引擎
- 包含机器狗模型引用

### go2w.sdf
- 简化的机器狗模型
- 四足机器人基本结构
- 配置关节限制和物理属性
- Gazebo Garden兼容的插件配置

### gazebo_garden.launch.py
- 完整的仿真启动文件
- 配置环境变量和路径
- 启动Gazebo Garden和ROS 2桥接
- 包含RViz可视化

### bridge.yaml
- Gazebo Garden与ROS 2的通信桥接配置
- 定义主题映射关系
- 支持关节状态、姿态、控制命令等

## 迁移注意事项

### 已完成的迁移
1. ✅ SDFormat版本升级
2. ✅ 物理引擎配置更新
3. ✅ 模型文件转换
4. ✅ 启动文件重写
5. ✅ 桥接配置创建

### 待处理事项
1. ⚠️ 自定义传感器插件需要重写
2. ⚠️ 复杂的网格模型可能需要优化
3. ⚠️ 高级物理效果需要重新配置

### 已知限制
- 当前模型为简化版本，用于验证迁移可行性
- 复杂的传感器和控制器需要额外开发
- 性能优化可能需要进一步调整

## 故障排除

### 常见问题

1. **Gazebo Garden未找到**
   ```bash
   # 检查安装
   gz --version
   # 如果未安装，参考官方文档安装
   ```

2. **模型加载失败**
   - 检查GAZEBO_MODEL_PATH环境变量
   - 验证模型文件路径是否正确

3. **ROS 2桥接问题**
   - 确认ros_gz_bridge包已安装
   - 检查bridge.yaml配置是否正确

## 下一步计划

1. **功能完善**
   - 添加更详细的机器狗模型
   - 集成传感器和控制器
   - 优化物理仿真参数

2. **性能优化**
   - 测试不同物理引擎性能
   - 优化模型复杂度
   - 调整仿真参数

3. **集成测试**
   - 与现有ROS 2节点集成
   - 验证导航和控制功能
   - 性能基准测试

## 技术支持

如有问题，请参考：
- [Gazebo Garden官方文档](https://gazebosim.org/docs)
- [ROS 2 Gazebo集成文档](https://github.com/gazebosim/ros_gz)
- [SDFormat规范](http://sdformat.org/)