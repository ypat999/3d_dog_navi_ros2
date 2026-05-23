import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'step_height_threshold',
            default_value='0.15',
            description='Height difference threshold to detect a step (m)'
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='3.0',
            description='Lookahead distance for sub-goal selection (m)'
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.5',
            description='Distance to segment end to consider it reached (m)'
        ),
        DeclareLaunchArgument(
            'speed_scale_step',
            default_value='0.3',
            description='Speed scale factor when on steps (0..1)'
        ),
        DeclareLaunchArgument(
            'speed_scale_flat',
            default_value='1.0',
            description='Speed scale factor on flat ground (0..1)'
        ),
        DeclareLaunchArgument(
            'step_zone_ahead',
            default_value='2.0',
            description='Forward distance to scan for upcoming steps (m)'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame id'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='Robot base frame id'
        ),

        Node(
            package='pct_path_adapter',
            executable='pct_path_adapter_node',
            name='pct_path_adapter',
            output='screen',
            parameters=[{
                'step_height_threshold': LaunchConfiguration('step_height_threshold'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'speed_scale_step': LaunchConfiguration('speed_scale_step'),
                'speed_scale_flat': LaunchConfiguration('speed_scale_flat'),
                'step_zone_ahead': LaunchConfiguration('step_zone_ahead'),
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
            }],
            remappings=[
                ('/pct_path', '/pct_path'),
                ('/plan', '/plan'),
                ('/goal_pose', '/goal_pose'),
            ],
        ),
    ])
