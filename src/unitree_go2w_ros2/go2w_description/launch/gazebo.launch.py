from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_go2w = FindPackageShare("go2w_description")
    xacro_file = PathJoinSubstitution([pkg_go2w, "urdf", "go2w_description.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_go2w, "config", "rviz_config.rviz"])
    controller_yaml = PathJoinSubstitution([pkg_go2w, "config", "joint_controller.yaml"])

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="default.world",
        description="World file name to load from the worlds directory"
    )

    
    # （可选）阻止Gazebo从互联网自动下载模型（加速本地加载）
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ""
    
    
    os.environ['GAZEBO_MODEL_PATH'] = '/usr/share/gazebo-11/models'
    os.environ['GAZEBO_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'

    # Set the path to the WORLD model files. Is to find the models inside the models folder in go2w_description package
    gazebo_models_path = os.path.join(get_package_share_directory('go2w_description'), 'models')
    local_models_path = os.path.expanduser('~/.gazebo/models')
    
    # Set GAZEBO_MODEL_PATH to include local models only, disable online retrieval
    model_paths = [local_models_path, gazebo_models_path]
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(model_paths)

    # Set GAZEBO_RESOURCE_PATH for shader libraries
    gazebo_resource_path = '/usr/share/gazebo-11'
    if 'GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['GAZEBO_RESOURCE_PATH'] = os.environ['GAZEBO_RESOURCE_PATH'] + ':' + gazebo_resource_path
    else:
        os.environ['GAZEBO_RESOURCE_PATH'] = gazebo_resource_path



    # 添加GPU渲染优化环境变量
    mesa_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA'
    )
    
    gazebo_gpu_rendering = SetEnvironmentVariable(
        name='GAZEBO_GPU_RENDERING',
        value='1'
    )



    gzserver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("gazebo_ros"), "/launch", "/gzserver.launch.py"
            ]),
            launch_arguments={
                "world": PathJoinSubstitution([
                    pkg_go2w, "worlds", "Building.world"
                ]),
                'extra_gazebo_args': '--disable-online-model-retrieval --disable-audio',
            }.items()
        )
    gzclient_launch = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
                "use_sim_time": True
            }
        ]
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "go2w",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.8",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         controller_yaml,
    #         {"use_sim_time": True}
    #     ],
    #     output="screen"
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", controller_yaml],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        mesa_adapter,
        gazebo_gpu_rendering,
        declare_world,
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_node,
        TimerAction(
            period=20.0,
            actions=[spawn_entity_node,
                    joint_state_broadcaster_spawner,
                    joint_group_effort_controller_spawner,
                    joint_group_velocity_controller_spawner,]
        ),
        # ros2_control_node,
        
        rviz_node
    ])
