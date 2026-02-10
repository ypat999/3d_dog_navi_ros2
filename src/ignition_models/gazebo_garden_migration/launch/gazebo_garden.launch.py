#!/usr/bin/env python3
"""
Gazebo Garden 启动文件
用于启动转换后的机器狗仿真环境
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # 设置Gazebo Garden环境变量
    env_vars = [
        # 禁用在线模型下载
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        
        # 设置模型路径
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
            f'{os.path.expanduser("~/.gazebo/models")}:'
            f'{os.path.dirname(__file__)}/../models:'
            f'/usr/share/gazebo/models'
        ),
        
        # 设置资源路径
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
            f'{os.path.dirname(__file__)}/../:'
            f'/usr/share/gazebo'
        ),
        
        # 设置插件路径
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
            f'/usr/lib/x86_64-linux-gnu/gazebo/plugins:'
            f'/opt/ros/humble/lib'
        )
    ]
    
    # Gazebo Garden进程
    world_file = os.path.join(os.path.dirname(__file__), '../worlds/Building.world')
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen',
        shell=True
    )
    
    # ROS 2桥接节点 - 用于Gazebo Garden与ROS 2通信
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': os.path.join(os.path.dirname(__file__), '../config/bridge.yaml')
        }],
        output='screen'
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(os.path.dirname(__file__), 
                                                  '../models/go2w/go2w.sdf')).read(),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # RViz2可视化
    rviz_config = os.path.join(os.path.dirname(__file__), '../config/rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        *env_vars,
        gazebo_process,
        bridge_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])