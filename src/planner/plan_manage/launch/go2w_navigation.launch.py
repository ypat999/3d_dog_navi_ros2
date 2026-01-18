import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfiguration definitions for Go2W robot
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    
    # Go2W robot initial position in Building world
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    
    # Navigation targets (including stairs and multi-level navigation)
    target_x = LaunchConfiguration('target_x', default=20.0)
    target_y = LaunchConfiguration('target_y', default=20.0)
    target_z = LaunchConfiguration('target_z', default=1.0)
    
    # Go2W robot ID and odometry topic
    robot_id = LaunchConfiguration('robot_id', default='go2w')
    odom_topic = LaunchConfiguration('odom_topic', default='odom')
    
    # DeclareLaunchArgument definitions
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x, description='Initial x position of the robot')
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial y position of the robot')
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z, description='Initial z position of the robot')
    
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x position')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y position')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z position')
    
    robot_id_cmd = DeclareLaunchArgument('robot_id', default_value=robot_id, description='ID of the Go2W robot')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # Include advanced parameters with GROUND MODE enabled for Go2W
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': robot_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            
            # Go2W-specific parameters for ground navigation
            'max_vel': str(1.5),           # Reduced max velocity for stability
            'max_acc': str(2.0),           # Reduced acceleration for legged robot
            'planning_horizon': str(10.0), # Extended planning horizon for complex terrain
            'flight_type': str(2),         # Ground mode flight type
            
            # Waypoint configuration for 3D navigation
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            
            # ========== GROUND MODE PARAMETERS ==========
            'enable_ground_mode': 'True',      # ENABLE GROUND MODE for Go2W
            'xy_extend': str(3),               # Reduced XY extend for ground navigation
            'z_extend': str(2),                # Increased Z extend for stairs
            'z_penalty_weight': str(1.5),      # Higher penalty for Z movement
            'xy_gradient_weight': str(0.8),    # Lower gradient weight for smoother ground movement
            # ========== END OF GROUND MODE PARAMETERS ==========
        }.items()
    )

    return LaunchDescription([
        map_size_x_cmd,
        map_size_y_cmd,
        map_size_z_cmd,
        init_x_cmd,
        init_y_cmd,
        init_z_cmd,
        target_x_cmd,
        target_y_cmd,
        target_z_cmd,
        robot_id_cmd,
        odom_topic_cmd,
        
        advanced_param_include,
    ])