import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mask_yaml = '/home/ar/team_ws/maps/keepout_mask_newmap.yaml'  # ★7/4 실사용 마스크로 교정(구맵 yaml이 기본이던 footgun)

    # 1) 마스크를 OccupancyGrid로 퍼블리시하는 Map Server
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'map',
            'topic_name': '/keepout_filter_mask',
            'yaml_filename': mask_yaml,
        }],
    )

    # 2) 마스크값→코스트 변환정보를 알려주는 Costmap Filter Info Server
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'type': 0,                       # 0 = keepout/preferred lanes
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    # 3) 위 두 노드를 활성화하는 lifecycle 매니저
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    return LaunchDescription([
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager,
    ])
