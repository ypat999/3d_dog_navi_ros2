import launch
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'topic_name',
            default_value='/low_level_joint_group_effort_controller/joint_trajectory',
            description='Name of the topic to publish joints'
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(get_package_share_directory('go2w_control'), 'src', 'joint_publisher_IHM.py'),
                LaunchConfiguration('topic_name')
            ],
            name='joint_publisher',
            output='screen'
        )
    ])
