#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_wheelchair = get_package_share_directory('wheelchair')

    description_package_name = "wheelchair"
    install_dir = get_package_prefix(description_package_name)
    gazebo_models_path = os.path.join(pkg_wheelchair, 'models')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = install_dir + '/share' + ':' + gazebo_models_path

    # Launch Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items()
    )

    # Include wheelchair launch
    wheelchair = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheelchair, 'launch', 'wheelchair.launch.py'),
        )
    )

    return LaunchDescription([
        ign_gazebo,
        wheelchair
    ])
