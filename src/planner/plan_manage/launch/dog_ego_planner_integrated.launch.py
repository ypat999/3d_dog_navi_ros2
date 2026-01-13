import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义参数
    drone_id = LaunchConfiguration('drone_id', default='0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_gazebo = LaunchConfiguration('enable_gazebo', default='true')
    
    # 地图参数
    map_size_x = LaunchConfiguration('map_size_x', default='50.0')
    map_size_y = LaunchConfiguration('map_size_y', default='25.0')
    map_size_z = LaunchConfiguration('map_size_z', default='2.0')
    
    # 声明参数
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time')
    enable_rviz_arg = DeclareLaunchArgument('enable_rviz', default_value=enable_rviz, description='Enable RViz')
    enable_gazebo_arg = DeclareLaunchArgument('enable_gazebo', default_value=enable_gazebo, description='Enable Gazebo simulation')
    
    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size Z')
    
    # 机器狗仿真启动 - 使用hybrid_controller.launch.py
    dog_simulation_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(
                 get_package_share_directory('go2w_control'),
                 'launch',
                 'hybrid_controller.launch.py'
             )
         ),
         launch_arguments={
             'world': 'default.world'
         }.items()
     )
    
    # 机器狗传感器数据直接使用 - 根据实际话题列表
    # 机器狗实际输出：/lidar/scan (PointCloud2类型) 和 /odom (里程计)
    
    # FAST_LIO2 里程计节点 - 使用机器狗激光雷达数据生成高精度里程计
    fast_lio2_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio2_dog_odom',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'config',
                'dog_lidar.yaml'
            ]),
            {
                'use_sim_time': use_sim_time,
                'common.lid_topic': '/lidar/scan',  # 直接使用机器狗的点云数据
                'common.imu_topic': '/imu/data',    # 机器狗IMU话题
                'publish.tf_child_frame_id': ['drone_', drone_id, '_base_link'],
                'publish.publish_odometry_with_covariance': True,
                'publish.publish_tf': True
            }
        ],
        remappings=[
            ('Odometry', ['drone_', drone_id, '_fast_lio_odom']),  # 发布fast_lio里程计
            ('cloud_registered', ['drone_', drone_id, '_fast_lio_cloud'])
        ]
    )
    
    # EgoPlanner启动
    ego_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ego_planner'),
                'launch',
                'advanced_param.launch.py'
            )
        ),
        launch_arguments={
                'drone_id': drone_id,
                'map_size_x_': map_size_x,
                'map_size_y_': map_size_y,
                'map_size_z_': map_size_z,
                'odometry_topic': ['drone_', drone_id, '_fast_lio_odom'],  # 使用fast_lio2生成的高精度里程计
                'cloud_topic': ['drone_', drone_id, '_fast_lio_cloud'],  # 使用fast_lio2处理后的点云数据
                'camera_pose_topic': 'camera_pose',
                'depth_topic': 'depth_image',
                # 目标点设置
                'point_num': '4',
                'point0_x': '5.0',
                'point0_y': '0.0',
                'point0_z': '0.5',
                'point1_x': '10.0',
                'point1_y': '5.0',
                'point1_z': '0.5',
                'point2_x': '15.0',
                'point2_y': '0.0',
                'point2_z': '0.5',
                'point3_x': '20.0',
                'point3_y': '5.0',
                'point3_z': '0.5',
                # 规划参数
                'max_vel': '2.0',
                'max_acc': '3.0',
                'planning_horizon': '7.5',
                'flight_type': '2',
                'use_distinctive_trajs': 'True',
                # 地面模式参数（适合机器狗）
                'enable_ground_mode': 'True',
                'xy_extend': '5',
                'z_extend': '1',
                'z_penalty_weight': '1.2',
                'xy_gradient_weight': '1.0'
            }.items()
    )
    
    # 轨迹服务器
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz配置
    rviz_config_file = os.path.join(
        get_package_share_directory('ego_planner'),
        'rviz',
        'dog_ego_planner.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz)
    )
    
    # 目标点发布节点（用于在RViz中设置目标点）
    goal_publisher_node = Node(
        package='ego_planner',
        executable='goal_publisher.py',
        name=['drone_', drone_id, '_goal_publisher'],
        output='screen',
        remappings=[
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point'])
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'goal_x': 5.0},
            {'goal_y': 0.0},
            {'goal_z': 0.5},
            {'publish_rate': 1.0}
        ]
    )
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(drone_id_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(enable_rviz_arg)
    ld.add_action(enable_gazebo_arg)
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    
    # 添加节点和启动文件
    ld.add_action(dog_simulation_launch)
    ld.add_action(fast_lio2_node)  # 添加fast_lio2里程计节点
    ld.add_action(ego_planner_launch)
    ld.add_action(traj_server_node)
    ld.add_action(rviz_node)
    ld.add_action(goal_publisher_node)
    
    return ld