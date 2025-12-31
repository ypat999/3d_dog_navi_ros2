# RViz 3D 导航目标工具  

随着技术的不断进步，**3D 导航** 需求变得越来越重要。这里提供了一个来自 **Willow Garage** 的旧工具，支持 **3D 导航目标** 的设置。该代码提取自这个 [仓库](https://github.com/HKUST-Aerial-Robotics/plan_utils)，更多详细信息请访问该链接。

## 🔧 编译方法  

此软件包与其他 **ROS** 软件包类似，只需克隆到你的工作空间并进行编译。

### 📥 克隆仓库  

- **HTTPS 克隆**：

```bash
git clone https://github.com/LiiXZ/rviz-3d-nav-goal-tool.git
```

### 🛠️ 编译  

```bash
catkin_make
```

## 🚀 使用方法  

1. **避免与 2D 导航目标冲突**  
   - 先点击 **"减号"（minus）** 图标 **移除 2D 导航目标** 。  
   - 然后点击 **"加号"（plus）** 图标 **选择 3D 导航目标** 。

2. **发送 3D 导航目标**  
   - **左键单击** 选择目标点并设定方向。  
   - **不要松开鼠标**，然后 **右键单击** 以调整目标的高度。  

## 📚 参考  

更多信息请参考：  
🔗 [3D Navigation 官方仓库](https://github.com/ros-planning/3d_navigation)
