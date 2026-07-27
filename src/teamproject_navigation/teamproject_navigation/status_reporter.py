#!/usr/bin/env python3
"""
status_reporter.py
로봇 상태(위치, 속도, 배터리, FSM 상태)를 1Hz로 발행.
★NAV_REPORT(7/14): 관제 GUI(성엽님 요청분)용 /robotN/nav_report(String/JSON, 2Hz+상태변화 즉시) 추가.
  — 회신_관제GUI_상태데이터_0713.md 스펙 그대로: 소문자 스네이크 필드 + 대문자 enum + updated_at(ISO 8601)

데이터 출처:
  - 위치 x, y, yaw       : TF map->base_footprint 에서 lookup (map 프레임. /odom 아님)
  - 속도 linear/angular  : /robotN/odom twist
  - battery             : /robotN/battery_state 의 percentage (변환 없이 그대로)
  - status / pause_reason : /robotN/state (patrol_commander 발행) 구독
  - current_target_wp    : /robotN/state 3번째 필드 (patrol_commander compute_target_wp)
  - nav_progress (JSON)  : patrol_commander 내부 스냅샷 — 랩 노드열/ESCAPE/전방거리/goal결과
  - /amcl_pose           : 공분산 → localization_state / quality (루트 네임스페이스)
  - /initialpose         : 수신 이력 → initial_pose_set (자동주입·RViz 클릭 둘 다 잡힘)
  - lifecycle get_state  : /amcl, /planner_server, /controller_server (5s 폴링, 비동기)

명명 규칙(팀): snake_case, 함수는 동사 시작.
"""
import json
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

import tf2_ros
from tf2_ros import TransformException

from lifecycle_msgs.srv import GetState

from teamproject_interfaces.msg import RobotStatus

# 전방 감속 시작 거리(m) — patrol_commander RS_FRONT_SLOW 와 동일값 (표시용 판정에만 사용)
FRONT_SLOW_DIST = 0.40


class StatusReporter(Node):
    def __init__(self):
        super().__init__('status_reporter')

        # --- 파라미터 (실행 시 -p robot_id:=2 로 변경) ---
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 1.0)
        # ★NAV_REPORT 파라미터
        self.declare_parameter('nav_report_rate', 2.0)          # 약속: 2Hz
        self.declare_parameter('map_id', 'factory_map_52x52')   # 신맵 고정값
        self.declare_parameter('route_id', 'patrol_v14')
        self.declare_parameter('route_name', 'factory_lap')

        self.robot_id = self.get_parameter('robot_id').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        nav_report_rate = self.get_parameter('nav_report_rate').value

        # --- 상태 저장소 ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_battery = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.current_state = 'IDLE'
        self.pause_reason = ''
        self.current_target_wp = -1
        self.have_pose = False  # TF를 한 번이라도 받았는지

        # ★NAV_REPORT 저장소
        self.nav_progress = {}          # patrol_commander JSON 스냅샷(마지막 수신값)
        self.amcl_cov = None            # (cov_xx, cov_yy, cov_yaw) — 수신 전 None=INITIALIZING
        self.initial_pose_set = False
        self.lifecycle_states = {'amcl': 'UNKNOWN',
                                 'planner_server': 'UNKNOWN',
                                 'controller_server': 'UNKNOWN'}
        self.obstacle_detected_at = None   # 전방 감속거리 진입 시각(ISO) — 벗어나면 None

        # --- TF buffer/listener (map->base_footprint lookup용) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 구독 (상대경로: namespace로 robotN 분리) ---
        self.create_subscription(Odometry, 'odom', self.update_velocity_from_odom, 10)
        self.create_subscription(BatteryState, 'battery_state', self.update_battery, 10)

        # /state 는 patrol_commander가 전이 시에만 발행 → latched(transient_local)로 받아
        # 늦게 떠도 최신 상태를 즉시 확보
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, 'state', self.update_state, state_qos)
        # ★NAV_REPORT: patrol_commander 내부 스냅샷(같은 latched QoS)
        self.create_subscription(String, 'nav_progress', self.update_nav_progress, state_qos)
        # AMCL 공분산 (Nav2 는 루트 네임스페이스 — 절대경로).
        # AMCL 발행이 latched(transient_local)라 QoS 를 맞춰야 '정지 중 재기동'에도
        # 마지막 pose 공분산을 즉시 받는다(volatile 구독이면 로봇이 움직일 때까지 INITIALIZING 고정).
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.update_amcl_pose, state_qos)
        # initialpose 수신 이력 = 초기위치 설정됨 (자동주입/수동 클릭 모두 이 토픽 경유)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.mark_initial_pose, 10)

        # ★NAV_REPORT: lifecycle 상태 폴링 클라이언트 (비동기, 5s 주기 — 블로킹 금지)
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.lifecycle_states
        }
        self.create_timer(5.0, self.poll_lifecycle_states)

        # --- 발행 ---
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.create_timer(1.0 / publish_rate, self.publish_status)
        # ★NAV_REPORT 발행 (2Hz + 상태변화 시 즉시)
        self.nav_report_pub = self.create_publisher(String, 'nav_report', 10)
        self.create_timer(1.0 / nav_report_rate, self.publish_nav_report)

        self.get_logger().info(
            f'StatusReporter up | robot_id={self.robot_id} '
            f'tf={self.map_frame}->{self.base_frame} | nav_report {nav_report_rate}Hz'
        )

    # ----------------------------------------------------------------
    def update_velocity_from_odom(self, msg):
        # 위치는 odom에서 쓰지 않는다(odom 프레임이라 map과 어긋남).
        # odom에서는 '속도'만 사용.
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def update_battery(self, msg):
        # 펌웨어 percentage 그대로 사용 (전압 변환 없음).
        battery_pct = msg.percentage
        if battery_pct is not None and not math.isnan(battery_pct):
            self.current_battery = battery_pct * 100.0 if battery_pct <= 1.0 else battery_pct

    def update_state(self, msg):
        # 형식: "STATE|reason|index" 3필드 (구버전 "STATE", "PAUSED|reason"도 안전 처리)
        raw_state = msg.data.strip()
        parts = raw_state.split('|')
        prev_state = self.current_state
        self.current_state = parts[0].strip()
        self.pause_reason = parts[1].strip() if (len(parts) >= 2 and self.current_state == 'PAUSED') else ''
        if len(parts) >= 3:
            try:
                self.current_target_wp = int(parts[2].strip())
            except ValueError:
                pass
        # ★NAV_REPORT: 상태변화 시 즉시 1회 발행 (2Hz 타이머와 별개)
        if self.current_state != prev_state:
            self.publish_nav_report()

    def update_nav_progress(self, msg):
        try:
            self.nav_progress = json.loads(msg.data)
        except (ValueError, TypeError):
            pass   # 손상 메시지는 버리고 직전 스냅샷 유지

    def update_amcl_pose(self, msg):
        cov = msg.pose.covariance
        self.amcl_cov = (cov[0], cov[7], cov[35])   # xx, yy, yaw-yaw

    def mark_initial_pose(self, msg):
        self.initial_pose_set = True

    def poll_lifecycle_states(self):
        for name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda f, n=name: self.store_lifecycle_state(n, f))

    def store_lifecycle_state(self, name, future):
        try:
            self.lifecycle_states[name] = future.result().current_state.label
        except Exception:
            pass   # 폴링 실패는 직전 값 유지 (표시용이라 무해)

    # ----------------------------------------------------------------
    def update_pose_from_tf(self):
        """map->base_footprint TF에서 map 프레임 x, y, yaw를 읽는다."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        self.current_x = transform.transform.translation.x
        self.current_y = transform.transform.translation.y

        rotation = transform.transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_pose = True

    def build_status_message(self):
        """현재 저장된 값으로 RobotStatus 메시지를 만든다."""
        msg = RobotStatus()
        msg.robot_id = int(self.robot_id)
        msg.x = float(self.current_x)
        msg.y = float(self.current_y)
        msg.yaw = float(self.current_yaw)
        msg.status = self.current_state
        msg.battery = float(self.current_battery)
        msg.linear_vel = float(self.linear_vel)
        msg.angular_vel = float(self.angular_vel)
        msg.pause_reason = self.pause_reason if self.current_state == 'PAUSED' else ''
        msg.current_target_wp = int(self.current_target_wp)
        return msg

    def publish_status(self):
        self.update_pose_from_tf()
        self.status_pub.publish(self.build_status_message())

        if not self.have_pose:
            self.get_logger().warn(
                'map->base_footprint TF 아직 없음 — 위치 (0,0) 발행 중 '
                '(LOCALIZING/2D Pose Estimate 확인)',
                throttle_duration_sec=5.0,
            )

    # ================================================================
    # ★NAV_REPORT — 관제 GUI 상태 데이터 (회신 문서 스펙)
    # ================================================================
    def derive_localization(self):
        """AMCL 공분산 → localization_state / quality / scan_match_state(근사 파생값)."""
        if self.amcl_cov is None:
            return {'localization_state': 'INITIALIZING', 'amcl_state': self.lifecycle_states['amcl'],
                    'initial_pose_set': self.initial_pose_set,
                    'localization_quality': 0.0, 'scan_match_state': 'UNKNOWN'}
        cov_xx, cov_yy, cov_yaw = self.amcl_cov
        localized = cov_xx < 0.25 and cov_yy < 0.25 and cov_yaw < 0.3
        quality = max(0.0, min(1.0, 1.0 - (cov_xx + cov_yy) / 0.5))
        scan_match = 'GOOD' if quality >= 0.7 else ('WEAK' if quality >= 0.4 else 'BAD')
        return {
            'localization_state': 'LOCALIZED' if localized else 'LOST',
            'amcl_state': self.lifecycle_states['amcl'],
            'initial_pose_set': self.initial_pose_set,
            'localization_quality': round(quality, 3),
            'scan_match_state': scan_match,   # AMCL 직접 지표 없음 — quality 파생(회신 문서 🟡근사)
        }

    def derive_nav2(self):
        """lifecycle + FSM → nav2_state / goal_result / replan_count."""
        fsm = self.current_state
        planner = self.lifecycle_states['planner_server']
        controller = self.lifecycle_states['controller_server']
        if planner != 'active' or controller != 'active':
            nav2_state = 'INACTIVE' if planner != 'UNKNOWN' else 'UNKNOWN'
        elif fsm == 'PAUSED':
            nav2_state = 'PAUSED'
        elif fsm == 'STUCK':
            nav2_state = 'ERROR'
        elif fsm in ('MOVING_TO_EVENT', 'RETURNING_TO_CHARGER') \
                or self.nav_progress.get('detour_active'):
            nav2_state = 'NAVIGATING'   # Nav2 goal in flight 인 국면
        else:
            nav2_state = 'ACTIVE'
        return {
            'nav2_state': nav2_state,
            'planner_state': planner,
            'controller_state': controller,
            'goal_result': self.nav_progress.get('goal_result', ''),
            'replan_count': self.nav_progress.get('replan_count', 0),
        }

    def build_route(self):
        """랩 노드열 → waypoints 배열(COMPLETED/CURRENT/PENDING) + route_state."""
        lap = self.nav_progress.get('lap')
        leg = self.nav_progress.get('leg', 0)
        nodes_xy = self.nav_progress.get('nodes_xy', {})
        fsm = self.current_state
        if fsm == 'STUCK':
            route_state = 'FAILED'
        elif not lap:
            route_state = 'NONE'
        else:
            route_state = 'ACTIVE'
        waypoints = []
        if lap:
            for i, node in enumerate(lap):
                xy = nodes_xy.get(str(node), [None, None])
                if i <= leg:
                    wp_status = 'COMPLETED'
                elif i == leg + 1:
                    # 구간 실패 국면(재시도/탈출/교착)이면 향하던 노드를 FAILED 로 표기
                    wp_status = ('FAILED' if fsm in ('RETRYING', 'ESCAPE', 'STUCK')
                                 else 'CURRENT')
                else:
                    wp_status = 'PENDING'
                waypoints.append({'waypoint_id': node, 'sequence': i + 1,
                                  'x': xy[0], 'y': xy[1], 'status': wp_status})
        return {
            'route_id': self.get_parameter('route_id').value,
            'route_name': self.get_parameter('route_name').value,
            'route_state': route_state,
            'current_target_wp': self.nav_progress.get('target_node', self.current_target_wp),
            'current_wp_index': (leg + 1) if lap else -1,
            'total_waypoints': len(lap) if lap else 0,
            'resume_node': self.nav_progress.get('resume_node'),
            'waypoints': waypoints,
        }

    def build_obstacle(self):
        """전방 라이다 최소거리 → obstacle_state / distance / 추정 x,y(전방 투영 근사)."""
        front = self.nav_progress.get('front_dist')
        fsm = self.current_state
        if fsm == 'OBSTACLE_WAITING':
            state = 'BLOCKED'
        elif front is not None and front < FRONT_SLOW_DIST:
            state = 'SLOWDOWN'
        else:
            state = 'NONE'
        # detected_at: 감속거리 진입 순간 기록, 벗어나면 리셋
        if state in ('SLOWDOWN', 'BLOCKED'):
            if self.obstacle_detected_at is None:
                self.obstacle_detected_at = self.make_timestamp()
        else:
            self.obstacle_detected_at = None
        obstacle_x = obstacle_y = None
        if front is not None and state != 'NONE' and self.have_pose:
            # 전방 콘 중심 방향으로 투영한 근사 좌표 (라이다 개별 빔 좌표 아님)
            obstacle_x = round(self.current_x + front * math.cos(self.current_yaw), 3)
            obstacle_y = round(self.current_y + front * math.sin(self.current_yaw), 3)
        return {
            'obstacle_state': state,
            'obstacle_type': 'UNKNOWN' if state != 'NONE' else None,
            'obstacle_distance': front,
            'obstacle_x': obstacle_x,
            'obstacle_y': obstacle_y,
            'detected_at': self.obstacle_detected_at,
        }

    def build_recovery(self):
        """ESCAPE 국면 → recovery_state / behavior (회신 문서 매핑 그대로)."""
        fsm = self.current_state
        escape_phase = self.nav_progress.get('escape_phase')
        if fsm == 'ESCAPE':
            state = 'RUNNING'
            behavior = {'SPIN': 'SPIN', 'DRIVE': 'BACK_UP'}.get(escape_phase, 'SPIN')
        elif fsm == 'RETRYING':
            state = 'RUNNING'
            behavior = 'REPLAN'
        elif fsm == 'OBSTACLE_WAITING':
            state = 'RUNNING'
            behavior = 'WAIT'
        else:
            state = 'IDLE'
            behavior = None
        return {
            'recovery_state': state,
            'recovery_behavior': behavior,
            'recovery_retry_count': self.nav_progress.get('escape_attempts', 0),
        }

    def make_timestamp(self):
        return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')

    def build_message(self):
        """사람이 읽는 한 줄 요약(회신 문서 '전 메시지 공통 message' 필드)."""
        fsm = self.current_state
        if fsm == 'PAUSED' and self.pause_reason:
            return f'일시정지: {self.pause_reason}'
        if fsm == 'ESCAPE':
            return f'탈출 복구 중 (시도 {self.nav_progress.get("escape_attempts", 0)}회)'
        if fsm == 'OBSTACLE_WAITING':
            return '전방 장애물 대기 중'
        if fsm == 'STUCK':
            return '주행 불가 — 관제 확인 필요'
        target = self.nav_progress.get('target_node', self.current_target_wp)
        if fsm == 'PATROLLING' and target not in (None, -1):
            return f'순찰 중 — 노드 {target} 이동'
        return fsm

    def publish_nav_report(self):
        try:
            report = {
                'robot_id': int(self.robot_id),
                'map_id': self.get_parameter('map_id').value,
                'fsm_state': self.current_state,
                'pause_reason': self.pause_reason,
                'pose': {'x': round(self.current_x, 3), 'y': round(self.current_y, 3),
                         'yaw': round(self.current_yaw, 3)} if self.have_pose else None,
                'battery': round(self.current_battery, 1),
                'localization': self.derive_localization(),
                'nav2': self.derive_nav2(),
                'route': self.build_route(),
                'obstacle': self.build_obstacle(),
                'recovery': self.build_recovery(),
                'message': self.build_message(),
                'updated_at': self.make_timestamp(),
            }
            msg = String()
            msg.data = json.dumps(report, ensure_ascii=False)
            self.nav_report_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'nav_report 발행 실패(무해): {e}',
                                   throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = StatusReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
