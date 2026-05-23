from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    workspace_src = "/home/ywj/git/3d_dog_navi_ros2/src"
    ignition_models_path = os.path.join(
        workspace_src, "ignition_models", "gazebo_garden_migration"
    )

    world_dir = os.path.join(ignition_models_path, "worlds")
    model_dir = os.path.join(ignition_models_path, "models")

    pkg_go2w_control = get_package_share_directory("go2w_control")
    pkg_go2w_description = get_package_share_directory("go2w_description")

    bridge_config = os.path.join(pkg_go2w_control, "config", "ignition_bridge.yaml")

    robot_sdf = os.path.join(model_dir, "go2w", "go2w_rl.sdf")

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="Building.world",
        # default_value="rubicon.sdf",
        description="World file name",
    )

    declare_world_name = DeclareLaunchArgument(
        name="world_name",
        default_value="tower",
        # default_value="challenge",
        description="World name in SDF file",
    )

    declare_robot_name = DeclareLaunchArgument(
        name="robot_name",
        default_value="go2w",
        description="Robot model name in Ignition Gazebo",
    )

    declare_verbose = DeclareLaunchArgument(
        name="verbose",
        default_value="3",
        description="Gazebo verbosity level (0-4)",
    )

    declare_x = DeclareLaunchArgument(
        name="x", default_value="-6.0", description="Initial x position"
    )

    declare_y = DeclareLaunchArgument(
        name="y", default_value="7.0", description="Initial y position"
    )

    declare_z = DeclareLaunchArgument(
        name="z", default_value="1.5", description="Initial z position"
    )

    declare_camera_x = DeclareLaunchArgument(
        name="camera_x", default_value="-10.0", description="Camera x position"
    )

    declare_camera_y = DeclareLaunchArgument(
        name="camera_y", default_value="-10.0", description="Camera y position"
    )

    declare_camera_z = DeclareLaunchArgument(
        name="camera_z", default_value="3.0", description="Camera z position"
    )

    gz_sim_resource_path = ":".join([model_dir, world_dir])

    env_vars = [
        SetEnvironmentVariable("DISPLAY", ":0"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_sim_resource_path),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            "/home/ywj/git/3d_dog_navi_ros2/install/gz_ros2_control/lib"
            ":/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins"
            ":/opt/ros/humble/lib",
        ),
    ]

    def generate_gazebo_command(context, *args, **kwargs):
        world_file = LaunchConfiguration("world").perform(context)
        world_path = os.path.join(world_dir, world_file)
        return [
            ExecuteProcess(
                cmd=[
                    "gz", "sim", "--force-version", "7",
                    "-r", "--headless-rendering", world_path,
                ],
                output="screen",
                shell=False,
            )
        ]

    gazebo_process = OpaqueFunction(function=generate_gazebo_command)

    def generate_spawn_command(context, *args, **kwargs):
        world_name = LaunchConfiguration("world_name").perform(context)
        robot_name = LaunchConfiguration("robot_name").perform(context)
        x = LaunchConfiguration("x").perform(context)
        y = LaunchConfiguration("y").perform(context)
        z = LaunchConfiguration("z").perform(context)
        return [
            ExecuteProcess(
                cmd=[
                    "gz", "service",
                    "-s", f"/world/{world_name}/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "30000",
                    "--req",
                    f'sdf_filename: "{robot_sdf}" '
                    f'name: "{robot_name}" '
                    f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}',
                ],
                output="screen",
                shell=False,
            )
        ]

    spawn_robot = OpaqueFunction(function=generate_spawn_command)

    # def generate_camera_command(context, *args, **kwargs):
    #     camera_x = LaunchConfiguration("camera_x").perform(context)
    #     camera_y = LaunchConfiguration("camera_y").perform(context)
    #     camera_z = LaunchConfiguration("camera_z").perform(context)
    #     camera_cmd = (
    #         f"gz camera -c gzclient_camera "
    #         f"--pos {camera_x} {camera_y} {camera_z}"
    #     )
    #     return [
    #         ExecuteProcess(
    #             cmd=["bash", "-c", camera_cmd],
    #             output="screen",
    #             shell=False,
    #         )
    #     ]

    # set_camera = OpaqueFunction(function=generate_camera_command)

    xacro_file = os.path.join(
        pkg_go2w_description, "urdf", "go2w_description_gz.urdf.xacro"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [FindExecutable(name="xacro"), " ", xacro_file]
                ),
                "use_sim_time": True,
            }
        ],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[
            {
                "robot_name": "go2w",
                "gazebo_model_name": "go2w_gazebo",
            }
        ],
    )

    rl_sim_node = Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    ignition_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ignition_bridge",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    gz_pose_relay = Node(
        package="topic_tools",
        executable="relay",
        name="gz_pose_relay",
        parameters=[{"use_sim_time": True}],
        arguments=["/go2w/pose", "/odom"],
        output="screen",
    )

    spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    imu_relay = Node(
        package="topic_tools",
        executable="relay",
        name="imu_relay",
        parameters=[{"use_sim_time": True}],
        arguments=["/imu/data", "/imu"],
        output="screen",
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    odom_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_world_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "world"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


    livox_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="livox_frame_to_lidar_tf",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "livox_frame", "go2w/livox_frame/mid360_lidar",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            *env_vars,
            declare_world,
            declare_world_name,
            declare_robot_name,
            declare_verbose,
            declare_x,
            declare_y,
            declare_z,
            declare_camera_x,
            declare_camera_y,
            declare_camera_z,
            gazebo_process,
            TimerAction(period=3.0, actions=[spawn_robot]),
            # TimerAction(period=4.0, actions=[set_camera]),
            robot_state_publisher_node,
            param_node,
            TimerAction(period=4.5, actions=[ignition_bridge]),
            TimerAction(period=5.0, actions=[imu_relay]),
            TimerAction(period=5.0, actions=[gz_pose_relay]),
            TimerAction(
                period=6.0, actions=[spawner_joint_state_broadcaster]
            ),
            map_to_odom_tf,
            odom_to_world_tf,
            world_to_base_tf,
            livox_frame_tf,
        ]
    )
