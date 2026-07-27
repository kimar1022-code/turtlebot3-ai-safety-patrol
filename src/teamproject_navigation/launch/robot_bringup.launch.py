#!/usr/bin/env python3
"""robot_bringup.launch.py — 로봇에서 실행: bringup + 카메라 + republish(압축)"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_bringup = FindPackageShare('turtlebot3_bringup')

    in_topic = '/camera/image_raw'
    out_topic = '/camera/image_raw/compressed'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tb3_bringup, 'launch', 'robot.launch.py'])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tb3_bringup, 'launch', 'camera.launch.py'])
            ),
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='camera_republish',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', in_topic),
                ('out/compressed', out_topic),
            ],
            output='screen',
        ),
    ])
