# unitree_go2w_ros2

This repository allows the user to use ROS2 to create a digital twin of the Unitree Go2W in Gazebo.

## Quick Start

The project was tested with ROS2 Humble and Ubuntu 22.04. 

1. **Clone the repository**:
    ```bash
        git clone https://github.com/Sam-Mag1/unitree_go2w_ros2.git
        cd unitree_go2w_ros2
    ```
2. **Install dependencies**:
    ```bash
        sudo apt-get install python3-rosdep
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    ```
3. **Build and source the workspace**:
    ```bash
        colcon build --symlink-install
        source install/setup.bash
    ```
4. **Launch the simulation with Low Level or High Level Controls**:
    - For Low Level Control:
    ```bash
        ros2 launch go2w_control gazebo_with_low_level.launch.py
    ```
    - For High Level Control:
    ```bash
        ros2 launch go2w_control gazebo_with_high_level.launch.py world:=outdoor.world
    ```
    The available worlds are `default.world`, `outdoor.world`, and `playground.world`.
    
    You can disable rviz2 by adding `rviz:=false` to the launch command.

From this point, you have the Rviz2 interface open, which shows the robot model and the joint states, and the Gazebo simulation running with the Unitree Go2W model.

You can control the robot using an GUI with sliders. The robot is controlled in position for the legs, and in velocity for the wheels.

## __/!\\ Important Note /!\\__
The Hihg-Level control is not perfect yet, and include linear motion on X and Y axes, and angular motion on Z axis. 

Sometimes, the robot may spawn before the controller is ready, which can lead to unexpected behavior (robot "jumping" due to collisions bugs) even before Gazebo is opened. If the robot is not on Origin Point of the Gazebo world, you can reset the pose of the robot by using the option "Edit > Reset Model Pose" in Gazebo.

With the Lidar and Camera added to the robot, the simulation is sometime slow (Real Time factor Low). If you have two GPUs, check that the simulation is running on the GPU with the best performance (you can check with the command `prime-select query`). If this is always too slow, contact me and I will add an option to disable the Lidar and Camera in the simulation to improve performance.

## To-Do List

- [x] Spawn the robot in Gazebo and link Rviz2 to the simulation
- [x] Be able to control the robot in effort mode
- [x] Add a very basic GUI to test the robot control
- [x] Be able to control the robot in position mode (without effort limitation)
- [x] Be able to control the robot in position mode (with an effort limitation)
- [x] Add IMU to the robot
- [x] High-Level Motion Control:
  - [x] Linear motion on X axis
  - [x] Linear motion on Y axis
  - [x] Angular motion on Z axis
- [x] Add Lidar and Camera to the robot
- [ ] SLAM integration

*This To-Do list is not exhaustive and will be updated as the project progresses.*

## How it works

