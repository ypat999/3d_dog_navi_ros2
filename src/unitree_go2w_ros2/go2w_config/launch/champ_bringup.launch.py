import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('go2w_config').find('go2w_config')
    description_pkg = FindPackageShare('go2w_description')

    joints_config = os.path.join(
        this_package, 'config', 'joints', 'joints.yaml'
    )
    gait_config = os.path.join(
        this_package, 'config', 'gait', 'gait.yaml'
    )
    links_config = os.path.join(
        this_package, 'config', 'links', 'links.yaml'
    )

    model_path = PathJoinSubstitution(
        [description_pkg, 'urdf', 'go2w_description.urdf.xacro']
    )

    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py']
    )

    base_frame = "base"

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name', 
            default_value='go2w',
            description='Set robot name for multi robot'
        ),

        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='hardware_connected', 
            default_value='false',
            description='Set to true if connected to a physical robot'
        ),

        DeclareLaunchArgument(
            name='joint_controller_topic', 
            default_value='champ_joint_group_effort_controller/joint_trajectory',
            description='Topic for joint controller'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("sim"),
                "description_path": model_path,
                "base_link_frame": base_frame,
                "robot_name": LaunchConfiguration("robot_name"),
                "gazebo": LaunchConfiguration("sim"),
                "rviz": LaunchConfiguration("rviz"),
                "joint_hardware_connected": LaunchConfiguration("hardware_connected"),
                "publish_joint_control": "true",
                "publish_foot_contacts": "true",
                "close_loop_odom": "true",
                "joint_controller_topic": LaunchConfiguration("joint_controller_topic"),
                "joints_map_path": joints_config,
                "links_map_path": links_config,
                "gait_config_path": gait_config
            }.items(),
        )
    ])
