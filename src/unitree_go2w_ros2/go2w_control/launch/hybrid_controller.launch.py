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
        
        # Gazebo odom真值转发节点 - 将Gazebo中的机器狗真实位姿转发为ROS2 odom话题
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_odom_bridge',
            arguments=[
                '/model/go2w/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '--ros-args', '-p', 'use_sim_time:=true'
            ],
            output='screen'
        ),
        
        # 重命名Gazebo odom真值话题到标准odom话题
        Node(
            package='topic_tools',
            executable='relay',
            name='gz_odom_relay',
            parameters=[{'use_sim_time': True}],
            arguments=['/model/go2w/odometry', '/odom_gazebo'],
            output='screen'
        ),
        
        # 添加静态tf变换 - 将Gazebo世界坐标系映射到ROS2世界坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_gz_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'gz_world'],
            output='screen'
        ),
    ])