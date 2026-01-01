from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='Spiral',
        description='Name of the scene to use'
    )
    
    pct_planner_node = Node(
        package='pct_planner_ros2',
        executable='pct_planner_node',
        name='pct_planner',
        parameters=[
            {'scene': LaunchConfiguration('scene')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        scene_arg,
        pct_planner_node
    ])