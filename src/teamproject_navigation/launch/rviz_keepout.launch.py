import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ============================================================================
# rviz_keepout.launch.py
#   RViz + KeepoutFilter(newmap) 를 한 번에 실행. (사용자 요청: 필터+RViz 합치기)
#
# ⚠️ 중복 금지 철칙 (지키지 않으면 코스트맵 흔들림 / rviz 중복):
#   · KeepoutFilter 노드(filter_mask_server 등)는 시스템 어디서든 "딱 1벌"만.
#     → 이 launch를 쓸 땐 별도 keepout_filter_newmap.launch.py 를 절대 같이 켜지 말 것.
#   · RViz도 1개만. 지금 rviz는 nav2 bringup(navigation2.launch.py)에서 뜸.
#     → 이 launch로 rviz를 띄우려면 nav2를 rviz 없이 띄우거나, 기존 rviz를 먼저 끌 것.
#
# 권장 사용 순서:
#   ① 기존 keepout / 여분 rviz 모두 종료  (ros2 node list 로 중복 0 확인)
#   ② nav2 는 rviz 없이:  ros2 launch turtlebot3_navigation2 navigation2.launch.py ... rviz:=False
#      (rviz 인자가 없으면 그냥 두고, 대신 이 launch의 rviz를 끄고 nav2 rviz만 쓰기)
#   ③ ros2 launch teamproject_navigation rviz_keepout.launch.py
#   ④ ros2 node list | sort | uniq -d   → 중복 0 이어야 정상
#
# 마스크만 keepout 하고 rviz는 nav2것 그대로 쓰고 싶으면:  use_rviz:=false
# ============================================================================

MASK_YAML = '/home/ar/team_ws/maps/keepout_mask_newmap.yaml'
DEFAULT_RVIZ = '/home/ar/turtlebot3_ws/install/turtlebot3_navigation2/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz'
# ★UDPCAM: PC측 UDP 카메라 수신 브리지(Pi가 UDP로 쏘는 JPEG → /robot1/camera/image_raw/compressed)
#   패키지 실행파일이 아니라 단독 스크립트라 ExecuteProcess로 실행. use_camera_bridge:=false 로 끌 수 있음.
CAM_BRIDGE = '/home/ar/team_ws/aruco_docking/udp_camera_bridge.py'

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_cfg = LaunchConfiguration('rviz_config')

    # --- KeepoutFilter (newmap) : keepout_filter_newmap.launch.py 와 100% 동일 ---
    filter_mask_server = Node(
        package='nav2_map_server', executable='map_server', name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'map',
            'topic_name': '/keepout_filter_mask',
            'yaml_filename': MASK_YAML,
        }],
    )
    costmap_filter_info_server = Node(
        package='nav2_map_server', executable='costmap_filter_info_server', name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters', output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    # --- RViz (use_rviz:=false 로 끌 수 있음 — nav2 rviz를 이미 쓰는 경우) ---
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # --- UDP 카메라 브리지 (PC측, use_camera_bridge:=false 로 끌 수 있음) ---
    #   Pi의 turtlebot_udp_camera_sender 가 쏘는 UDP JPEG 를 받아
    #   /robot1/camera/image_raw/compressed 로 재발행. 도킹/ID2검출 노드가 이걸 구독.
    camera_bridge = ExecuteProcess(
        cmd=['python3', CAM_BRIDGE, '--ros-args',
             '-p', ['port:=', LaunchConfiguration('cam_port')]],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera_bridge')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='RViz도 이 launch에서 띄울지 (false면 keepout만)'),
        DeclareLaunchArgument('rviz_config', default_value=DEFAULT_RVIZ,
                              description='RViz 설정파일(.rviz) 경로'),
        DeclareLaunchArgument('use_camera_bridge', default_value='true',
                              description='PC측 UDP 카메라 브리지도 이 launch에서 띄울지'),
        DeclareLaunchArgument('cam_port', default_value='5007',
                              description='UDP 카메라 수신 포트 (sender와 동일해야 함)'),
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager,
        rviz,
        camera_bridge,
    ])
