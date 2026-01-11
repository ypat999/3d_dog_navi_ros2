import launch
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory('go2w_control'), 'src', 'high_level_publisher_IHM.py')],
            name='cmd_vel_publisher',
            output='screen'
        )
    ])
