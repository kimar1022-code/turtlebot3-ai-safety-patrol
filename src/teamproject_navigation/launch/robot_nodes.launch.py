import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    namespace = PythonExpression(["'/robot' + str(", robot_id, ")"])
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='1',
                              description='로봇 ID (1/2/3)'),
        # ★7/9: 기본 false로 강등 — CLEAN_START가 aruco_dock_detector를 상시 띄우므로
        #   매니저가 DOCKING 진입 시 또 스폰 → 동명 노드 2개 + /detected_dock_pose 이중발행.
        #   상시 검출을 안 쓰는 구성에서만 true로 켤 것.
        DeclareLaunchArgument('use_dock_detector_mgr', default_value='false',
                              description='도킹 검출노드 매니저(PC) on/off — DOCKING 상태에서만 검출노드 켬. CLEAN_START 상시검출과 배타'),
        # ★2026-07-04 사용자 승인: 도킹 게이트 기본 ON (재시작마다 param set 하던 것 영구화).
        #   도킹 없이 순찰만: use_dock_executor:=false / 여러바퀴: max_laps:=3 등
        DeclareLaunchArgument('use_dock_executor', default_value='true',
                              description='랩 끝 도킹 사이클 on/off (Pi executor)'),
        DeclareLaunchArgument('dock_x', default_value='0.0', description='도크 map x (로봇별)'),
        DeclareLaunchArgument('dock_y', default_value='0.0', description='도크 map y (로봇별)'),
        DeclareLaunchArgument('dock_yaw', default_value='0.0', description='도크 map yaw[rad] (로봇별)'),
        DeclareLaunchArgument('route_graph', default_value='~/team_ws/maps/patrol_graph.geojson',
                              description='순찰 route 그래프 (로봇별 — 프리도킹 노드가 다름)'),
        DeclareLaunchArgument('max_laps', default_value='1',
                              description='이 바퀴수 후 도킹 주차(0=무한)'),
        # ★2026-07-04: ID2 마커추종(yaw메모리 정렬 완성판) 런처 통합 — 순찰 중 ID2 발견→파견
        #   →도착 후 기억좌표로 시선정렬. 끄려면 use_id2_follow:=false
        DeclareLaunchArgument('use_id2_follow', default_value='true',
                              description='ID2 마커추종 검출/파견/yaw정렬 노드 on/off'),
        Node(
            package='teamproject_navigation',
            executable='status_reporter',
            namespace=namespace,
            name='status_reporter',
            parameters=[{'robot_id': robot_id}],
            remappings=[
                ('odom', '/odom'),
                ('battery_state', '/battery_state'),
            ],
            output='screen',
        ),
        Node(
            package='teamproject_navigation',
            executable='patrol_commander',
            namespace=namespace,
            name='patrol_commander',
            parameters=[{
                'robot_id': robot_id,
                # ★도킹 게이트 launch 인자로 주입(재시작마다 param set 리셋되던 것 영구화)
                'use_dock_executor': ParameterValue(
                    LaunchConfiguration('use_dock_executor'), value_type=bool),
                'max_laps': ParameterValue(
                    LaunchConfiguration('max_laps'), value_type=int),
                # ★DOCK_POSE(7/14, 멀티로봇): 로봇별 도크 map 좌표 (로봇1 기본 0,0,0)
                'dock_x': ParameterValue(LaunchConfiguration('dock_x'), value_type=float),
                'dock_y': ParameterValue(LaunchConfiguration('dock_y'), value_type=float),
                'dock_yaw': ParameterValue(LaunchConfiguration('dock_yaw'), value_type=float),
                'route_graph': ParameterValue(LaunchConfiguration('route_graph'), value_type=str),
            }],
            remappings=[
                ('navigate_to_pose', '/navigate_to_pose'),
                ('battery_state', '/battery_state'),  # commander가 배터리 직접 구독 (글로벌 토픽)
                ('collision_monitor_state', '/collision_monitor_state'),  # ★OBSTACLE_BRIDGE: 라이다 정지 신호(글로벌)
            ],
            output='screen',
        ),
        # ★NAMESPACE(7/7): 관제 수동조작 게이트 — 서버가 /robotN/cmd_vel 발행 →
        #   MANUAL_CONTROL 상태에서만 루트 /cmd_vel(TwistStamped)로 통과.
        #   기존엔 /robotN/cmd_vel 구독자가 없어 수동조작 무반응 + 상태 게이트 부재였음.
        #   서버가 TwistStamped를 보내면 input_stamped:=true (기본 Twist).
        #   ★7/8 실측: 서버 factory_robot_bridge가 /robotN/cmd_vel을 TwistStamped로 발행 →
        #     input_stamped=True 고정(False면 타입불일치로 게이트가 수동명령 미수신=무반응). 실주행 검증됨.
        Node(
            package='teamproject_navigation',
            executable='manual_cmd_gate',
            namespace=namespace,
            name='manual_cmd_gate',
            parameters=[{'robot_id': robot_id, 'input_stamped': True}],
            output='screen',
        ),
        # ★DOCK: 도킹 검출노드 자동 ON/OFF 매니저(PC 로컬 스크립트).
        #   /robotN/state == DOCKING일 때만 aruco_dock_detector 켜고, 벗어나면 끔.
        #   게이트 OFF면 DOCKING 상태가 안 생겨 → 아무 동작 안 함(무해).
        #   ★NAMESPACE(7/7): robot_id 주입(/robot1 하드코딩 제거)
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/team_ws/aruco_docking/dock_detector_manager.py'),
                 '--ros-args', '-p',
                 PythonExpression(["'robot_id:=' + str(", robot_id, ")"])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_dock_detector_mgr')),
        ),
        # ★ID2 추종: 카메라 정적TF(장착값 근사, id2/도킹 검출 공용) + 검출·파견·yaw정렬 노드.
        #   TF는 id2 게이트와 무관하게 항상 무해(정적 1회 latch).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            # 7/6 카메라 이설 실측: 라이다앞면+8cm(=바퀴축 기준 -3.2+1.7+8=6.5cm, 라이다지름3.4), 바닥 16.3cm(-바퀴축 3.3 = z 0.13)
            arguments=['--x', '0.065', '--y', '0', '--z', '0.13',
                       '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'camera_optical_frame'],
            output='screen',
        ),
        # ★NAMESPACE(7/7): id2 검출기에 robot_id 기반 토픽/서비스 주입(/robot1 기본값 하드코딩 대체)
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/team_ws/aruco_docking/id2_follow_detector.py'),
                 '--ros-args', '-p', 'optical_frame:=camera_optical_frame',
                 '-p', PythonExpression(["'dispatch_service:=/robot' + str(", robot_id, ") + '/dispatch_to_event'"]),
                 '-p', PythonExpression(["'state_topic:=/robot' + str(", robot_id, ") + '/state'"]),
                 '-p', PythonExpression(["'image_topic:=/robot' + str(", robot_id, ") + '/camera/image_raw/compressed'"]),
                 '-p', PythonExpression(["'camera_info_topic:=/robot' + str(", robot_id, ") + '/camera/camera_info'"])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_id2_follow')),
        ),
    ])