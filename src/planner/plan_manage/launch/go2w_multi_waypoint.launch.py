import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfiguration definitions for Go2W robot with multi-level navigation
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    
    # Go2W robot ID and odometry topic
    robot_id = LaunchConfiguration('robot_id', default='go2w')
    odom_topic = LaunchConfiguration('odom_topic', default='odom')
    
    # DeclareLaunchArgument definitions
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    
    robot_id_cmd = DeclareLaunchArgument('robot_id', default_value=robot_id, description='ID of the Go2W robot')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # Include advanced parameters with GROUND MODE enabled for Go2W
    # Multi-waypoint configuration for testing stairs and multi-level navigation
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
            'max_vel': str(1.2),           # Conservative velocity for legged robot
            'max_acc': str(1.5),           # Conservative acceleration
            'planning_horizon': str(12.0), # Extended planning horizon for complex terrain
            'flight_type': str(2),         # Ground mode flight type
            
            # Multi-waypoint configuration for 3D navigation testing
            # Waypoints designed to test stairs and multi-level movement
            'point_num': str(4),
            
            # Waypoint 1: Ground level navigation
            'point0_x': str(10.0),
            'point0_y': str(5.0),
            'point0_z': str(0.0),
            
            # Waypoint 2: Stair climbing (elevation change)
            'point1_x': str(15.0),
            'point1_y': str(10.0),
            'point1_z': str(1.5),
            
            # Waypoint 3: Upper level navigation
            'point2_x': str(20.0),
            'point2_y': str(15.0),
            'point2_z': str(2.0),
            
            # Waypoint 4: Stair descending
            'point3_x': str(25.0),
            'point3_y': str(20.0),
            'point3_z': str(0.5),
            
            # ========== GROUND MODE PARAMETERS ==========
            'enable_ground_mode': 'True',      # ENABLE GROUND MODE for Go2W
            'xy_extend': str(4),               # Balanced XY extend
            'z_extend': str(3),                # Increased Z extend for stairs
            'z_penalty_weight': str(1.8),      # Higher penalty for Z movement (stability)
            'xy_gradient_weight': str(0.7),    # Lower gradient weight for smoother movement
            # ========== END OF GROUND MODE PARAMETERS ==========
        }.items()
    )

    return LaunchDescription([
        map_size_x_cmd,
        map_size_y_cmd,
        map_size_z_cmd,
        robot_id_cmd,
        odom_topic_cmd,
        
        advanced_param_include,
    ])