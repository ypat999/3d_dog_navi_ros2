# Livox MID360 + IMU仿真 可用于Fast LIO
A package to provide plug-in for [Livox Series LiDAR](https://www.livoxtech.com).

## Environment
我自己电脑测试的环境如下
- ROS(=Noetic)
- Gazebo11
- Ubuntu(20.04)

## Usage

**克隆仓库并编译**
```shell
cd ~/catkin_ws/src
git clone https://github.com/qiurongcan/Mid360_imu_sim.git
cd ..
catkin_make
```

**运行代码**
```shell
roslaunch livox_laser_simulation mid360_IMU_platform.launch
```
在`mid360_IMU_platform.launch`文件中，可以注释最后的rviz这几行，不显示rviz,也可以取消注释显示rviz  
可以手动在gazebo中添加物体【长方形、圆柱...】,达到一个更好的演示效果
```xml
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="30.0" />
  </node>

  <!-- RViz -->
  <!-- <arg name="rviz" default="true"/>
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find livox_laser_simulation)/rviz/livox_simulation.rviz"/> -->

```
**查看话题**
此时会有两个话题
```shell
/scan sensor_msgs/PointCloud
/livox/imu
```
这个雷达的数据类型Fast_LIO是没办法使用的，需要对其进行转换

**数据类型转化**
```shell
# 运行转换脚本
rosrun livox_laser_simulation pointcloud2livox.py
# 也可以直接运行python文件
cd Mid360_imu_sim/script
python3 pointcloud2livox.py
```
最后输出的话题为
```shell
/livox/imu
/livox/lidar2
```
之后用fast_lio订阅即可

## Parameters(only for display , and example by avia)

- laser_min_range: 0.1  // min detection range
- laser_max_range: 200.0  // max detection range
- horizontal_fov: 70.4   //°
- vertical_fov: 77.2    //°
- ros_topic: scan // topic in ros
- samples: 24000  // number of points in each scan loop
- downsample: 1 // we can increment this para to decrease the consumption

