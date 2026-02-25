from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    workspace_src = '/home/ywj/git/3d_dog_navi_ros2/src'
    ignition_models_path = os.path.join(workspace_src, 'ignition_models', 'gazebo_garden_migration')
    
    world_dir = os.path.join(ignition_models_path, 'worlds')
    model_dir = os.path.join(ignition_models_path, 'models')
    
    champ_bringup_launcher = os.path.join(
        get_package_share_directory('go2w_config'), 
        'launch', 'champ_bringup.launch.py'
    )
    
    pkg_share = get_package_share_directory('go2w_control')
    bridge_config = os.path.join(pkg_share, 'config', 'ignition_bridge.yaml')
    
    robot_sdf = os.path.join(model_dir, 'go2w', 'go2w.sdf')
    
    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="Building.world",
        description="World file name to load from ignition_models worlds directory"
    )
    
    declare_robot_name = DeclareLaunchArgument(
        name="robot_name",
        default_value="go2w",
        description="Robot model name in Ignition Gazebo"
    )
    
    declare_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="Launch RViz2"
    )
    
    declare_verbose = DeclareLaunchArgument(
        name="verbose",
        default_value="3",
        description="Gazebo verbosity level (0-4)"
    )
    
    declare_x = DeclareLaunchArgument(
        name="x",
        default_value="-10.0",
        description="Initial x position"
    )
    
    declare_y = DeclareLaunchArgument(
        name="y",
        default_value="10.0",
        description="Initial y position"
    )
    
    declare_z = DeclareLaunchArgument(
        name="z",
        default_value="0.6",
        description="Initial z position"
    )
    
    gz_sim_resource_path = ':'.join([
        model_dir,
        world_dir,
    ])
    
    env_vars = [
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_sim_resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/home/ywj/git/3d_dog_navi_ros2/install/gz_ros2_control/lib:/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins:/opt/ros/humble/lib'),
        
    ]
    
    def generate_gazebo_command(context, *args, **kwargs):
        world_file = LaunchConfiguration('world').perform(context)
        world_path = os.path.join(world_dir, world_file)
        
        return [ExecuteProcess(
            cmd=['gz', 'sim', '--force-version', '8', '-r', '--headless-rendering', world_path],
            output='screen',
            shell=False
        )]
    
    gazebo_process = OpaqueFunction(function=generate_gazebo_command)
    
    def generate_spawn_command(context, *args, **kwargs):
        robot_name = LaunchConfiguration('robot_name').perform(context)
        x = LaunchConfiguration('x').perform(context)
        y = LaunchConfiguration('y').perform(context)
        z = LaunchConfiguration('z').perform(context)
        
        return [ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/tower/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '30000',
                '--req',
                f'sdf_filename: "{robot_sdf}" name: "{robot_name}" pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
            ],
            output='screen',
            shell=False
        )]
    
    spawn_robot = OpaqueFunction(function=generate_spawn_command)
    
    return LaunchDescription([
        *env_vars,
        declare_world,
        declare_robot_name,
        declare_rviz,
        declare_verbose,
        declare_x,
        declare_y,
        declare_z,
        
        gazebo_process,
        
        TimerAction(
            period=3.0,
            actions=[
                spawn_robot,
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(champ_bringup_launcher),
                    launch_arguments={
                        'joint_controller_topic': '/joint_group_effort_controller/joint_trajectory',
                        'sim': 'true',
                        'rviz': LaunchConfiguration('rviz'),
                        'hardware_connected': 'false',
                        'orientation_from_imu': 'true'
                    }.items()
                ),
            ]
        ),
        
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='go2w_control',
                    executable='hybrid_motion_controller.py',
                    name='hybrid_motion_controller',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
        
        TimerAction(
            period=4.5,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ignition_bridge',
                    parameters=[{
                        'config_file': bridge_config,
                        'use_sim_time': True
                    }],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='topic_tools',
                    executable='relay',
                    name='gz_pose_relay',
                    parameters=[{'use_sim_time': True}],
                    arguments=['/go2w/pose', '/odom'],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_joint_state_broadcaster',
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=6.5,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_joint_group_velocity_controller',
                    arguments=['joint_group_velocity_controller'],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_joint_group_effort_controller',
                    arguments=['joint_group_effort_controller'],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),
        
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/joint_group_effort_controller/joint_trajectory',
                        'trajectory_msgs/msg/JointTrajectory',
                        '{joint_names: [FL_hip_joint, FL_thigh_joint, FL_calf_joint, FR_hip_joint, FR_thigh_joint, FR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint, RR_hip_joint, RR_thigh_joint, RR_calf_joint], points: [{positions: [0.0, 1.0143535137176514, -2.0291322589877523, 0.0, 1.0143535137176514, -2.0291322589877523, 0.0, 1.0143535137176514, -2.0291322589877523, 0.0, 1.0143535137176514, -2.0291322589877523], time_from_start: {sec: 0, nanosec: 100000000}}]}'
                    ],
                    output='screen',
                    shell=False
                ),
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
