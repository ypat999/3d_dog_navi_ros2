import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import sys, os


def generate_launch_description():
    # 首先导入全局配置

    # 正确导入global_config包
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
    
    package_path = get_package_share_directory('fast_lio')
    config_path = os.path.join(package_path, 'config')

    use_sim_time = DEFAULT_USE_SIM_TIME
    config_path = config_path
    config_file = 'mid360.yaml'

    # 导入全局配置
    from global_config import LIVOX_MID360_CONFIG
    
    livox_config_path = LIVOX_MID360_CONFIG
    lidar_mode = "ONLINE"
    if not ONLINE_LIDAR:
        lidar_mode = "OFFLINE"


    ld = LaunchDescription()


    # 在线模式：Livox雷达驱动
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": 'livox0000000001'},
        ],
        prefix=['taskset -c 4,5'],   # 绑定 CPU 4
        condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'ONLINE'")),
    )

    # # 离线模式：rosbag播放
    # from launch.actions import ExecuteProcess
    # rosbag_player = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'play', DEFAULT_BAG_PATH, '--qos-profile-overrides-path', DEFAULT_RELIABILITY_OVERRIDE, '--clock', '--rate', '1.0'],
    #     name='rosbag_player',
    #     output='screen',
    #     prefix=['taskset -c 4'],   # 绑定 CPU 4
    #     condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'OFFLINE'")),
    # )

    # 根据模式选择启动相应的节点
    # ld.add_action(rosbag_player)
    ld.add_action(livox_driver_node)


    # 创建FAST-LIO生命周期节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        prefix=['taskset -c 7'],   # 绑定 CPU 7
        output='screen',
        namespace='',
        # 启用生命周期管理
        arguments=['--ros-args', '--log-level', 'info'],
        # 配置为生命周期节点
        emulate_tty=True,
    )

    # 创建生命周期管理器节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_fastlio',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['fastlio_mapping'],
            'bond_timeout': 10.0,
            # 禁用bond心跳超时机制，避免与FAST-LIO节点连接失败
            'bond_disable_heartbeat_timeout': True,
        }],
        # 确保生命周期管理器能够正确管理FAST-LIO节点
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # 使用TimerAction添加延迟启动FAST-LIO节点，确保雷达数据就位
    from launch.actions import TimerAction
    ld.add_action(
        TimerAction(
            period=5.0,  # 延迟5秒启动FAST-LIO节点
            actions=[fast_lio_node, lifecycle_manager]
        )
    )
    

    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # ('/cloud_in', '/lio_sam/deskew/cloud_deskewed'),
            ('/cloud_in', '/cloud_registered_body'),
            ('/scan', '/scan'),
        ],
        parameters=[{
            'transform_tolerance': 0.1,
            'min_height': 0.1,           # 最小高度（过滤掉地面以下的点，调整为更紧的范围）
            'max_height': 1.5,            # 最大高度（过滤掉较高的点，限制在地面附近）
            'angle_min': -3.1,        # -180度
            'angle_max': 3.1,         # 180度
            # 将角度增量精确设置为 (angle_max - angle_min) / (691 - 1)
            # 原始地图中的激光束数量为 691，实际转换产生 690 时会触发 slam_toolbox 的长度校验错误。
            # 使用精确值可以避免舍入导致的“expected 691 / got 690”问题。
            'angle_increment': 0.00869347338,
            'scan_time': 0.1,             # 扫描时间
            
            'range_min': 0.3,             # 增加最小距离，过滤掉近距离噪声 (原0.8)
            'range_max': 40.0,             # 减少最大距离，避免远距离噪声影响 (原10.0)
            'use_inf': False,              # 是否使用无穷大值（布尔类型，不使用引号）
            
            'inf_epsilon': 40.0,           # 无穷大值的替代值
            
            # # QoS设置，确保与rviz2订阅者兼容
            # 'qos_overrides./scan.publisher.reliability': 'reliable',
            # 'qos_overrides./scan.publisher.depth': 10,
            
            # 其他参数
            'use_sim_time': use_sim_time,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为odom，确保laserscan保持水平，不随baselink倾斜
            'target_frame': 'livox_frame',
            'concurrency_level': 1,       # 处理并发级别
        }],
        output='screen',
        prefix=['taskset -c 5'],   # 绑定 CPU 5
    )
    ld.add_action(pointcloud_to_laserscan_node)



    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    # odom -> base_link (里程计到机器人基坐标系的静态变换)
    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_base_link)

    # base_link -> livox_frame (机器人基坐标系到雷达坐标系的静态变换)
    base_link_to_livox_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.1', '0', '0.1', '0', '0.0', '0', 'base_link', 'livox_frame'],
        output='screen'
    )
    ld.add_action(base_link_to_livox_frame_tf)

    # 根据模式添加相应的节点（按照LIO-SAM的逻辑）
    if RECORD_ONLY:
        # 仅录制模式：只启动雷达驱动
        return ld

    # 获取nav2_dog_slam包的路径
    nav2_dog_slam_path = get_package_share_directory('nav2_dog_slam')
    
    # 添加slam_toolbox节点（按照LIO-SAM的模式）
    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params')
    declare_slam_toolbox_params_cmd = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value=NAV2_DEFAULT_PARAMS_FILE,
        description='Full path to slam_toolbox parameters file'
    )
    
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {
                'use_sim_time': use_sim_time,
                # force-disable map publishing/updating to keep nav2 map_server as authoritative
                'map_update_interval': 1.0,
                'publish_occupancy_map': True,
                'use_map_saver': True
            }
        ],
        prefix=['taskset -c 5,6'],
        remappings=[('/scan', '/scan'), ('/odom', '/Odometry')]  # 使用FAST-LIO的odometry话题
    )
    
    # 添加octomap_server节点（按照LIO-SAM的模式）
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',                   # 地图坐标系
            'sensor_model/max_range': 100.0,     # 最大感测距离
            'sensor_model/min_range': 0.4,       # 最小感测距离
            'sensor_model/insert_free_space': True,
            'resolution': 0.05,                  # OctoMap 分辨率（5cm）
            'occupancy_min_z': -0.1,             # 投影高度下限
            'occupancy_max_z': 1.0,              # 投影高度上限
            'publish_2d_map': True,              # 输出2D occupancy grid
            'use_sim_time': use_sim_time,
        }],
        prefix=['taskset -c 4,5'],
        remappings=[
            ('/cloud_in', '/cloud_registered_body')  # 输入点云
        ]
    )

    # 根据模式添加相应的节点
    if BUILD_MAP:
        if BUILD_TOOL == 'slam_toolbox':
            # 建图模式 + slam_toolbox
            ld.add_action(declare_slam_toolbox_params_cmd)
            ld.add_action(
                TimerAction(
                    period=5.0,  # 延迟20秒启动slam_toolbox
                    actions=[slam_toolbox_node]
                )
            )
        else:
            # 建图模式：添加octomap server
            ld.add_action(
                TimerAction(
                    period=15.0,  # 延迟15秒启动octomap_server
                    actions=[octomap_server_node]
                )
            )
    else:
        # 非建图模式：只添加pointcloud_to_laserscan节点
        pass

    return ld