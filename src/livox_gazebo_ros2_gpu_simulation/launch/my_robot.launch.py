#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # （可选）阻止Gazebo从互联网自动下载模型（加速本地加载）
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ""

    # 声明启动参数
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run gazebo headless')
        
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Set to "true" to show verbose output')

    # 设置Gazebo环境变量
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11'
    )
    
    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models'
    )
    
    ogre_resource_path = SetEnvironmentVariable(
        name='OGRE_RESOURCE_PATH',
        value='/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'
    )
    
    
    # 添加GPU渲染优化环境变量
    mesa_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA'
    )
    
    gazebo_gpu_rendering = SetEnvironmentVariable(
        name='GAZEBO_GPU_RENDERING',
        value='1'
    )









    package_name = 'livox_gazebo_ros2_gpu_simulation'
    robot_name = 'my_robot'
    world_file_name = 'test_zone.world' #'bigHHH.world'
    use_sim_time = True

    pkg_livox_gazebo_ros2_gpu_simulation = get_package_share_directory(package_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Robot description using xacro
    robot_desc = Command(['xacro ', os.path.join(pkg_livox_gazebo_ros2_gpu_simulation, 'urdf', 'my_robot', 'my_robot.xacro')])
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_livox_gazebo_ros2_gpu_simulation, 'worlds', world_file_name),
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )
    
    # Robot State Publisher - 限制发布频率
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
            'publish_frequency': 1.0,  # 限制为30Hz
        }],
    )
    
    # Joint State Publisher - 限制发布频率
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 1.0,  # 限制为30Hz
        }],
    )
    
    # Spawn robot with a delay to ensure Gazebo is ready
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5'],
    )
    
    # RVIZ
    rviz_path = os.path.join(pkg_livox_gazebo_ros2_gpu_simulation, 'rviz', robot_name + '.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # 延迟启动spawn_entity节点，确保Gazebo完全启动
    delayed_spawn_entity = TimerAction(
        period=10.0,
        actions=[spawn_entity]
    )
    
    return LaunchDescription([
        # 声明启动参数
        gui_arg,
        verbose_arg,
        # 设置环境变量
        gazebo_resource_path,
        gazebo_plugin_path,
        gazebo_model_path,
        ogre_resource_path,
        mesa_adapter,
        gazebo_gpu_rendering,
        # 启动Gazebo
        gazebo_launch,
        # 启动机器人相关节点
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn_entity,
        # 启动RViz
        rviz,
    ])