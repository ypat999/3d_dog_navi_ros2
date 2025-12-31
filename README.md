# 3D 导航仿真项目

## 项目介绍
本项目主要用于在 Gazebo 仿真环境中验证 [PCT-planner](https://github.com/byangw/PCT_planner.git) 和 [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) 两种路径规划算法。项目中使用了 Unitree A1 机器人模型以及强化学习控制器 [chy2948331536/unitree_guide](https://gitee.com/chy2948331536/unitree_guide)，其中控制器和 PCT-planner 需要 CUDA 支持。

### 3D导航示例
<div align="center">
  <img src="/src/image/878355907.gif" width="800"/>
</div>


### 3D规划示例
<div align="center">
  <img src="/src/image/123141123.gif" width="800"/>
</div>

---

## 下载与依赖

### 依赖项安装
1. **libtorch**：下载 C++ 版本的 [libtorch](https://pytorch.org/)。
2. **PCT-planner**：将其放置在 `src` 文件夹外单独编译。
3. **ego-planner**：请参考其官方文档进行配置：[ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git)
4. **Fast-lio**：如需使用，可运行 `.auto.sh` 脚本自动发布 `livox/lidar` 和 `livox/imu` 话题，但需手动设置 `tf` 坐标关系。

---

## 安装步骤

### 1. 配置 libtorch 和 CUDA 路径
修改 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中的 `libtorch` 路径和 `CMAKE_CUDA_COMPILER` 路径。

### 2. 安装 ego-planner
请参考 [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) 官方文档安装相关依赖。

### 3. 编译项目
进入 ROS 工作空间并执行以下命令：
```bash
catkin_make
```

### 4. 安装 PCT-planner
请参考 [PCT-planner](https://github.com/byangw/PCT_planner.git) 官方文档安装依赖，然后使用以下命令进行编译：
```bash
cd planner/
./build_thirdparty.sh
./build.sh
```

---

## 使用说明

### 1. 启动 RL 控制器
由于 RL 控制器需要手柄，因此需先启动虚拟控制器：
```bash
sudo -s
source ./devel/setup.bash
rosrun unitree_guide virtual_joy.py
```

然后启动 Gazebo 仿真环境并运行控制器：
```bash
. auto.sh  # 等待 Unitree A1 机器人展开
./devel/lib/unitree_guide/junior_ctrl
```

在控制器中：
- 按键 **2**：站立
- 按键 **6**：切换为 RL 模式（此时接收 `cmd_vel` 消息）
- 再次按键 **2**：会闪退，需重新启动控制器

### 2. 启动 ego-planner
```bash
source ./devel/setup.bash
roslaunch ego_planner run_in_sim.launch  # 局部导航模块
roslaunch ego_planner ego_rviz.launch  # RVIZ 可视化
```

修改 `run_in_sim.launch` 文件中的 `flight_type` 参数可切换导航模式：
```xml
<!-- 1: 使用 2D Nav Goal 设置目标 -->
<arg name="flight_type" value="1" />
<!-- 3: 使用 move_base 的路径 -->
<arg name="flight_type" value="3" />
```

### 3. 启动 PCT-planner
进入 `PCT-planner` 文件夹并运行以下命令：
```bash
cd tomography/scripts/ 
python3 tomography.py --scene Building

cd planner/scripts/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:home/YOUR-NAME/3d-navi/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
python3 plan.py --scene Building
```

> ⚠️ **注意**：如果地图配置更改，需重新生成地图，路径规划器才会重新规划路径。

---

## 注意事项
1. 项目代码较为仓促，可能存在不规范或混乱的情况，敬请谅解。
2. 如发现问题或有任何建议，请及时提交 issue，便于改进。
3. 本项目基于 [PCT-planner](https://github.com/byangw/PCT_planner.git)、[ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) 和 [unitree_guide](https://gitee.com/chy2948331536/unitree_guide.git) 构建，仅限学习使用，禁止用于商业用途。

---

## 联系方式
- **Bilibili**：[https://space.bilibili.com/29152879](https://space.bilibili.com/29152879)
- **邮箱**：1906570332@qq.com

如果您觉得本项目对您有帮助，欢迎在 Gitee 上给我点个 **star**！谢谢支持！