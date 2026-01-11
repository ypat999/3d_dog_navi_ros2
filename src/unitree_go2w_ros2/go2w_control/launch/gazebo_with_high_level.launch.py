from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    simulation_launcher = os.path.join(get_package_share_directory('go2w_description'), 'launch', 'gazebo.launch.py')

    champ_bringup_launcher = os.path.join(get_package_share_directory('go2w_config'), 'launch', 'champ_bringup.launch.py')
    topic_name = '/joint_group_effort_controller/joint_trajectory'

    high_level_publisher_launcher = os.path.join(get_package_share_directory('go2w_control'), 'launch', 'high_level_publisher.launch.py')

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="default.world",
        description="World file name to load from the worlds directory"
    )

    return LaunchDescription([
        declare_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launcher),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_bringup_launcher),
            launch_arguments={'joint_controller_topic': topic_name}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(high_level_publisher_launcher)
        ),
    ])