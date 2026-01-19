import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    config_path = os.path.join(package_path, 'config')

    use_sim_time = True
    config_path = config_path
    config_file = 'mid360.yaml'

    ld = LaunchDescription()


    # 创建FAST-LIO生命周期节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': True}],
        output='screen',
    )

    # 使用TimerAction添加延迟启动FAST-LIO节点，确保雷达数据就位
    from launch.actions import TimerAction
    ld.add_action(
        TimerAction(
            period=5.0,  # 延迟5秒启动FAST-LIO节点
            actions=[fast_lio_node]
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
    )
    ld.add_action(pointcloud_to_laserscan_node)



    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': True}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    # odom -> base_footprint (里程计到机器人身体坐标系的静态变换)
    static_transform_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_footprint',
        parameters=[{'use_sim_time': True}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint'],
        output='screen'
    )
    # ld.add_action(static_transform_odom_to_base_footprint)


    return ld