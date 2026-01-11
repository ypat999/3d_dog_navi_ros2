from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_description')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    
    use_xacro = True

    urdf_path = os.path.join(pkg_share, 'urdf', 'go2w_description.urdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'go2w_description.urdf.xacro')

    robot_description = Command(['xacro ', xacro_path]) if use_xacro else open(urdf_path).read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'user_debug',
            default_value='false',
            description='Enable debug mode'
        ),

        # robot_description param loaded from URDF file content
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description},
                        {'publish_frequency': 1000.0}]
        ),

        # joint_state_publisher_gui node with use_gui = True
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}]
        ),

        # RViz2 with config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])