#!/usr/bin/env python3
"""
Gazebo Garden 启动文件 - 四足机器狗导航适配版
参考: livox_gazebo_garden/launch/livox_garden.launch.py
用于启动四足机器狗仿真环境，支持 Nav2 导航
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gazebo_garden_migration')

    # 设置 Gazebo Garden 环境变量
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            f'{os.path.expanduser("~/.gazebo/models")}:'
            f'{pkg_dir}/models:'
            f'{pkg_dir}/worlds:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
            f'{pkg_dir}:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            f'{pkg_dir}/worlds:'
            f'{pkg_dir}/models:'
            f'{pkg_dir}:'
            f'/usr/share/gz/gz-sim'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
            f'{pkg_dir}/lib:'
            '/opt/ros/humble/lib:'
            '${GZ_SIM_SYSTEM_PLUGIN_PATH}'),
    ]

    # World 文件
    world_file = 'Building.world'

    # Gazebo Garden 进程
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_dir, 'worlds', world_file)],
        output='screen',
    )

    # SDF 模型路径
    sdf_path = os.path.join(pkg_dir, 'models', 'go2w', 'go2w.sdf')
    
    # 读取机器人描述 (用于 robot_state_publisher)
    with open(sdf_path, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'publish_frequency': 10.0,
        }],
        output='screen',
    )

    # 使用 ros_gz_sim 生成实体
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-file', sdf_path,
            '-name', 'go2w',
            '-world', 'Building',
            '-x', '0.0', '-y', '0.0', '-z', '0.7',
            '-timeout', '60.0',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ROS-Gazebo 桥接节点
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # 静态 TF 发布器: livox_frame -> body/livox_frame/lidar
    # 用于兼容 LIO-SAM 的坐标系要求
    static_transform_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox_frame',
        parameters=[{'use_sim_time': True}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'livox_frame', 'go2w/livox_frame'],
        output='screen'
    )

    # 静态 TF 发布器: base_footprint -> body
    # 用于 Nav2 导航 (Nav2 期望 base_footprint)
    static_transform_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_footprint',
        parameters=[{'use_sim_time': True}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'body'],
        output='screen'
    )

    # RViz2 可视化
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Joint State Publisher (可选，用于调试)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': True, 'rate': 10.0}],
    #     output='screen',
    # )

    return LaunchDescription([
        *env_vars,
        gazebo_process,
        robot_state_publisher,
        # 延迟生成实体，等待 Gazebo 完全启动
        TimerAction(period=10.0, actions=[spawn_entity]),
        # 静态 TF 发布器
        static_transform_livox,
        static_transform_base_footprint,
        # 延迟启动桥接和 RViz
        TimerAction(period=15.0, actions=[
            bridge_node,
            rviz_node,
        ]),
    ])
