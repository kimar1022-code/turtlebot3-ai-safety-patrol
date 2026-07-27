#!/usr/bin/env python3
"""robot_bringup.launch.py — bringup(TF 전역) + 카메라(remap robot1 + 방향보정) + 도킹 실행노드
   use_camera:=false 로 카메라 없이 띄울 수 있음 (기본 true)
   use_dock_executor:=false 로 도킹 실행노드 없이 띄울 수 있음 (기본 true)"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_bringup = FindPackageShare('turtlebot3_bringup')

    use_camera = LaunchConfiguration('use_camera')
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='카메라 on/off (false면 카메라 없이 브링업)',
    )

    # ★DOCK: Pi 로컬 도킹/언도킹 실행노드 (patrol_commander가 /robot1/dock_cmd로 구동).
    #   상주하지만 IDLE에선 cmd_vel 미발행이라 순찰에 무영향. 실제 도킹 동작은
    #   patrol_commander의 use_dock_executor 파라미터가 True일 때만 명령이 온다.
    use_dock_executor = LaunchConfiguration('use_dock_executor')
    declare_use_dock_executor = DeclareLaunchArgument(
        'use_dock_executor',
        default_value='true',
        description='도킹 실행노드(pi_dock_executor) on/off',
    )
    dock_executor = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/pi_dock_executor.py')],
        output='screen',
        condition=IfCondition(use_dock_executor),
    )

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(use_camera),
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                parameters=[{
                    'camera': 0,
                    'orientation': 180,
                    'width': 320,
                    'height': 240,
                    'format': 'XRGB8888',
                }],
                remappings=[
                    ('/camera/image_raw', '/robot1/camera/image_raw'),
                    ('/camera/image_raw/compressed', '/robot1/camera/image_raw/compressed'),
                    ('/camera/camera_info', '/robot1/camera/camera_info'),
                ],
            ),
        ],
    )

    return LaunchDescription([
        declare_use_camera,
        declare_use_dock_executor,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tb3_bringup, 'launch', 'robot.launch.py'])
            ),
        ),
        camera_container,
        dock_executor,
    ])
