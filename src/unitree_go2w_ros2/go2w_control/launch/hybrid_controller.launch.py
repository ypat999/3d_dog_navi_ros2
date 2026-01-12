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
            launch_arguments={'joint_controller_topic': '/joint_group_effort_controller/joint_trajectory'}.items()
        ),
        # 混合运动控制器节点
        Node(
            package='go2w_control',
            executable='hybrid_motion_controller.py',
            name='hybrid_motion_controller',
            output='screen'
        ),
    ])