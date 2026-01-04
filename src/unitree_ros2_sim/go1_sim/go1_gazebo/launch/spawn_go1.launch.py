import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():
    # Disable audio completely to prevent ALSA errors
    os.environ['GAZEBO_AUDIO'] = '0'
    os.environ['ALSOFT_DRIVERS'] = 'null'
    
    world_file_name = LaunchConfiguration('world_file_name')
    urdf_file = LaunchConfiguration('urdf_file')
    

    world_file_name_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='Building.world' #'test_latest.world'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.xacro'
    )

    # Position and orientation
    # [X, Y, Z]
    position = [0, 0.0, 1.6]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_name = "GO1"
    
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/start_world.launch.py']),
    launch_arguments={'world_file_name': world_file_name}.items(),
    )
    
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # Use ExecuteProcess with delay to ensure Gazebo is fully started
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', robot_name,
             '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
             '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
             '-topic', '/robot_description'],
        output='screen'
    )

    odom_tf_publisher_node = Node(
        package='go1_navigation',
        executable='nav_tf_publisher',
        name='odom_transform_publisher',
        output='screen'
    )

    # static_map_publisher_node = Node(
    #     package='go1_navigation',
    #     executable='static_map_publisher',
    #     name='static_map_publisher',
    #     output='screen'
    # )

    # map_file_path = os.path.join(get_package_share_directory('go1_navigation'), 'map', 'map.yaml')

    # map_publisher_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'yaml_filename' : map_file_path}
    #     ]
    # )

    # map_publisher_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': map_file_path}],
    #     remappings=[('map', 'map')]
    # )
    
    # manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': False},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )

    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/controllers_go1.launch.py'])
    )

    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_description'), 'launch', 'go1_visualize.launch.py')
            ]),
    launch_arguments={'use_joint_state_publisher': 'False',
                      'use_sim_time': "True",
                      'urdf_file': urdf_file}.items(),
    )
    
    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_teleop'), 'launch', 'joystick.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )
    
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_navigation'), 'launch', 'nav_core.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )


    # create and return launch description object
    return LaunchDescription(
        [
            world_file_name_arg,
            urdf_file_arg,
            start_world,
            visualize_robot,
            odom_tf_publisher_node,
            # static_map_publisher_node
            # map_publisher_node,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #       target_action=map_publisher_node,
            #       on_exit=[manager_node],
            #     )
            # )
            # joystick, 
            # navigation,
            # Delay spawn_robot by 10 seconds to ensure Gazebo is fully started
            TimerAction(
                period=20.0,
                actions=[spawn_robot]
            ),
            # Delay controller launch by 15 seconds to ensure robot is spawned
            TimerAction(
                period=20.0,
                actions=[launch_ros2_control]
            )
        ]
    )