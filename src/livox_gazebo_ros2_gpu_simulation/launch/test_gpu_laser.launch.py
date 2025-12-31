#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 设置Gazebo环境变量
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        '/usr/share/gazebo-11'
    )
    
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        '/usr/share/gazebo-11/models'
    )
    
    ogre_resource_path = SetEnvironmentVariable(
        'OGRE_RESOURCE_PATH',
        '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'
    )
    
    # 添加GPU渲染优化环境变量
    mesa_adapter = SetEnvironmentVariable(
        'MESA_D3D12_DEFAULT_ADAPTER_NAME',
        'NVIDIA'
    )
    
    gazebo_gpu_rendering = SetEnvironmentVariable(
        'GAZEBO_GPU_RENDERING',
        '1'
    )



    
    package_name = 'livox_gazebo_ros2_gpu_simulation'
    robot_name = 'my_robot'
    world_file_name = 'bigHHH.world'
    use_sim_time = True
    gui = True

    pkg_livox_gazebo_ros2_gpu_simulation = get_package_share_directory(package_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gui_arg = LaunchConfiguration('gui', default=gui)
    



    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        #empty   standardrobots_factory
        launch_arguments={
            'world': os.path.join(pkg_livox_gazebo_ros2_gpu_simulation, 'worlds', world_file_name),
            'gui': gui_arg,
            'verbose': 'true'
        }.items()
    )
    
    # 启动激光雷达数据监听节点
    laser_listener = Node(
        package='livox_gazebo_ros2_gpu_simulation',
        executable='laser_listener',
        name='laser_listener',
        output='screen'
    )
    
    return LaunchDescription([
        # 设置环境变量
        gazebo_resource_path,
        gazebo_plugin_path,
        gazebo_model_path,
        ogre_resource_path,
        mesa_adapter,
        gazebo_gpu_rendering,
        # 启动参数和进程
        gazebo_launch,
        laser_listener,
    ])
