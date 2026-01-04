#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_go1_gazebo = get_package_share_directory('go1_gazebo')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "go1_description"
    install_dir = get_package_prefix(description_package_name)

    os.environ['GAZEBO_MODEL_PATH'] = '/usr/share/gazebo-11/models'
    os.environ['GAZEBO_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_go1_gazebo, 'models')
    local_models_path = os.path.expanduser('~/.gazebo/models')
    
    # Set GAZEBO_MODEL_PATH to include local models only, disable online retrieval
    model_paths = [local_models_path, install_dir + '/share', gazebo_models_path]
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(model_paths)
    
    # Disable online model database to prevent network retrieval
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Set GAZEBO_RESOURCE_PATH for shader libraries
    gazebo_resource_path = '/usr/share/gazebo-11'
    if 'GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['GAZEBO_RESOURCE_PATH'] = os.environ['GAZEBO_RESOURCE_PATH'] + ':' + gazebo_resource_path
    else:
        os.environ['GAZEBO_RESOURCE_PATH'] = gazebo_resource_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    print("GAZEBO RESOURCE PATH=="+str(os.environ["GAZEBO_RESOURCE_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'extra_gazebo_args': '--disable-online-model-retrieval --disable-audio'
        }.items()
    ) 
       
    world_file_name = LaunchConfiguration('world_file_name')

    world_file_name_launch_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='Building.world'  #'test_latest.world'
    )


    return LaunchDescription([
        world_file_name_launch_arg,
        DeclareLaunchArgument(
          'world',
        #   default_value=[os.path.join(pkg_go1_gazebo, 'worlds',  ), ''],
          default_value=[os.path.join(pkg_go1_gazebo, 'worlds'),'/', world_file_name,''],
          description='SDF world file'),
        gazebo,
    ])