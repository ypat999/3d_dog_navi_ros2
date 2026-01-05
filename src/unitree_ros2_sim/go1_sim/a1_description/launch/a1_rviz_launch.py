#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('a1_description')
    
    # Declare launch arguments
    user_debug = DeclareLaunchArgument(
        'user_debug',
        default_value='false',
        description='Enable user debug mode'
    )
    
    # Robot description
    robot_description_config = os.path.join(
        pkg_share,
        'xacro',
        'robot.xacro'
    )
    
    robot_description_cmd = f'xacro {robot_description_config} DEBUG:={LaunchConfiguration("user_debug")}'
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'publish_frequency': 1000.0
        }]
    )
    
    # Joint state publisher node (GUI version)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(pkg_share, 'launch', 'check_joint.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(user_debug)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld