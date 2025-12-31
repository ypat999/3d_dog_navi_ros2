# Unitree GO1 Simulation with ROS 2

This repository contains a simulation environment for the Unitree GO1 robot in Gazebo 11 and ROS 2, along with an interface for navigation. The functionality has been tested with ROS Humble on Ubuntu 22.04

## Dependencies:
- lcm (Needs to be built from source, instructions [here](https://lcm-proj.github.io/lcm/))
- [navigation2](https://github.com/ros-navigation/navigation2)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)

## Structure

The repository contains the following ROS 2 packages:
- `go1_sim`:
    - `go1_description`: Contains required xacro and config files to load the robot in simulation. Modified from [here](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go1_description).
    - `go1_gazebo`: Contains the gazebo world and required launch files to initialize the simulation.
    - `go1_navigation`: Contains scripts, launch and configuration files for interfacing with a navigation stack.

- `ros2_unitree_legged_controller`: Contains the implementation of unitree's control plugin in ROS 2. This has been ported from the ROS 1 plugin available [here](https://github.com/unitreerobotics/unitree_ros/tree/master/unitree_legged_control).

- `ros2_unitree_legged_msgs`: Contains unitree custom messages for interfacing, source [here.](https://github.com/unitreerobotics/unitree_ros2_to_real/tree/main/ros2_unitree_legged_msgs)

- `unitree_guide2`: Provides an interface between the navigation and control framework along with a state machine for different control modes. This has been ported to ROS 2 from [here](https://github.com/unitreerobotics/unitree_guide).


## Testing

After placing all the packages in a ROS 2 workspace and building it successfully, run the following in order:


1. *(window 1)* `ros2 launch go1_gazebo spawn_go1.launch.py`: This will load the simulation and initialize controllers.

2. *(window 2)* `ros2 run unitree_guide2 junior_ctrl`: This activates the interface and state machine. **Run this once the last controller plugin (*RL_calf_controller*) has loaded successfully**
    1. Press 2 to switch the robot to standing mode (fixed_stand)
    2. Press 5 to switch to move_base mode (robot accepts velocity commands in this mode)

## Interface

### Subscribed topics

**Velocity control interface:**
 The robot simulation is configured to navigate as per velocity commands received on the interface topic.
- Topic: `/cmd_vel`
- Message type: [geometry_msgs/Twist](https://docs.ros.org/en/ros2_packages/humble/api/geometry_msgs/interfaces/msg/Twist.html)


### Published topics

**Odometry data:**
Odometry data from the ground truth plugin
- Topic: `/odom`
- Message type: [nav_msgs/Odometry](https://docs.ros.org/en/humble/p/nav_msgs/interfaces/msg/Odometry.html)


**2D LiDAR data:**
2D lidar data from the gazebo sensor plugin
- Topic: `/scan`
- Message type: [sensor_msgs/LaserScan](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/interfaces/msg/LaserScan.html)


**Transforms:**
Transforms `odom` -> `base_link` and `base_link` -> `base_footprint` are provided
 - Topic: `/tf`
 - Message type: [tf2_msgs/TFMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html)
