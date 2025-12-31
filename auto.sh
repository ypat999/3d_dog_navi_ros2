#!/bin/bash
echo "Terminating gazebo and junior_ctrl processes..."
# 查找并关闭 gazebo 进程
pkill -f gazebo
if [ $? -eq 0 ]; then
    echo "Successfully terminated gazebo processes."
else
    echo "No gazebo processes found or failed to terminate."
fi

# 查找并关闭 junior_ctrl 进程
pkill -f junior_ctrl
if [ $? -eq 0 ]; then
    echo "Successfully terminated junior_ctrl processes."
else
    echo "No junior_ctrl processes found or failed to terminate."
fi

# Step 2: 进入上级目录并运行 catkin build
echo "Navigating to the parent directory and running catkin build..."
# cd ../..
# source 到环境中
source ./devel/setup.bash

if [ $? -ne 0 ]; then
    echo "Failed to navigate to the parent directory. Exiting."
    exit 1
fi

catkin_make -DCATKIN_WHITELIST_PACKAGES="unitree_guide"
if [ $? -ne 0 ]; then
    echo "catkin build failed. Please check for errors."
    exit 1
fi
echo "catkin build completed successfully."
# Step3: 启动 gazeboSim.launch 和 junior_ctrl
echo "Launching ROS nodes..."

# 启动 gazeboSim.launch
sleep 2s
#将地图模型添加到环境变量中
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find unitree_gazebo)/models
roslaunch unitree_guide gazeboSim.launch user_debug:=False rname:=a1&
LAUNCH_PID=$!
if [ $? -eq 0 ]; then
    echo "Successfully launched gazeboSim.launch (PID: $LAUNCH_PID)."
else
    echo "Failed to launch gazeboSim.launch. Exiting."
    exit 1
fi

#Step 4:打开雷达工作空间，并打开雷达fast-lio节点
# sleep 2s
# cd /home/cjh/livox_ws
# source ./devel/setup.bash
# roslaunch fast_lio mapping_mid360.launch&
# LAUNCH_PID=$!
# if [ $? -eq 0 ]; then
#     echo "Successfully launched mapping_mid360.launch (PID: $LAUNCH_PID)."
# else
#     echo "Failed to launch mapping_mid360.launch. Exiting."
#     exit 1
# fi

# Step 5: 返回到 unitree_guide 目录
# sleep 5s
# cd ~/unitree_ws
# # 启动 junior_ctrl
# ./devel/lib/unitree_guide/junior_ctrl
# CTRL_PID=$!
# if [ $? -eq 0 ]; then
#    echo "Successfully launched junior_ctrl (PID: $CTRL_PID)."
# else
#    echo "Failed to launch junior_ctrl. Exiting."
#    exit 1
# fi

#echo "All processes launched successfully."
