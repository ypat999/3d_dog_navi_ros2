from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    simulation_launcher = os.path.join(get_package_share_directory('go2w_description'), 'launch', 'gazebo.launch.py')

    joint_publisher_launcher = os.path.join(get_package_share_directory('go2w_control'), 'launch', 'joint_publisher.launch.py')

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
            PythonLaunchDescriptionSource(joint_publisher_launcher),
            launch_arguments={'topic_name': '/joint_group_effort_controller/joint_trajectory'}.items()
        ),
    ])