#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    package_name = "hik_camera"
    config_dir = get_package_share_directory(package_name)
    
    default_config_path = Path(config_dir) / "config" / "hik_camera_params.yaml"

    config_argument = DeclareLaunchArgument(
        name='config_path',
        default_value=str(default_config_path),
        description='Configuration file path for camera parameters'
    )
    
    rviz_flag = DeclareLaunchArgument(
        name='enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    rviz_config_argument = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='',
        description='Custom RViz configuration file'
    )

    camera_driver = Node(
        package=package_name,
        executable='hik_camera_node',
        name='camera_driver',
        output='screen',
        parameters=[LaunchConfiguration('config_path')]
    )

    rviz_visualizer = Node(
        package='rviz2',
        executable='rviz2',
        name='visualization_tool',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return LaunchDescription([
        config_argument,
        rviz_flag,
        rviz_config_argument,
        camera_driver,
        rviz_visualizer
    ])
