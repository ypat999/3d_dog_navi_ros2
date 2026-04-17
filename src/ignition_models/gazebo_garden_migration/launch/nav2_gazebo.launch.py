#!/usr/bin/env python3
"""
四足机器狗导航统一启动文件
启动 Gazebo 仿真 + Super-LIO SLAM + Nav2 导航
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gazebo_garden_migration')
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map file to load (empty for SLAM)'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    # 启动 Gazebo 仿真
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'gazebo_garden.launch.py')
        ])
    )

    # Super-LIO SLAM (需要确保 super_lio 包已安装)
    # 如果没有 super_lio，可以注释掉这部分
    try:
        super_lio_pkg = get_package_share_directory('super_lio')
        super_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(super_lio_pkg, 'launch', 'mapping.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'lidar_topic': '/livox/lidar',
                'imu_topic': '/livox/imu',
            }.items()
        )
    except Exception:
        print("super_lio package not found, skipping SLAM launch")
        super_lio_launch = None

    # Nav2 导航
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'map': LaunchConfiguration('map_file'),
        }.items()
    )

    # Hybrid Motion Controller (四足机器狗混合运动控制器)
    # 需要确保 go2w_control 包已安装
    try:
        go2w_control_pkg = get_package_share_directory('go2w_control')
        hybrid_controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(go2w_control_pkg, 'launch', 'hybrid_controller.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'true',
            }.items()
        )
    except Exception:
        print("go2w_control package not found, skipping hybrid controller launch")
        hybrid_controller_launch = None

    # 构建启动描述
    launch_actions = [
        declare_use_sim_time,
        declare_map_file,
        declare_nav2_params,
        gazebo_launch,
    ]

    # 延迟启动 SLAM (等待 Gazebo 完全启动)
    if super_lio_launch:
        launch_actions.append(
            TimerAction(period=20.0, actions=[super_lio_launch])
        )

    # 延迟启动混合运动控制器
    if hybrid_controller_launch:
        launch_actions.append(
            TimerAction(period=25.0, actions=[hybrid_controller_launch])
        )

    # 延迟启动 Nav2 (等待 SLAM 初始化)
    launch_actions.append(
        TimerAction(period=30.0, actions=[nav2_launch])
    )

    return LaunchDescription(launch_actions)
