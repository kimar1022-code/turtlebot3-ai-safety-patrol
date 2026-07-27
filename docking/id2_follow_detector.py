#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ID2 마커 추종 파견 노드 (PC에서 실행)
======================================
순찰 중 멀리 있는 ArUco **ID2**(DICT_4X4_50, 15cm)를 발견하면,
마커 **정면 30cm 앞** pose를 map 좌표로 계산해 patrol_commander의
기존 `dispatch_to_event` 서비스를 1회 호출한다.

그 뒤 흐름은 **검증된 FSM이 그대로 처리**한다:
  순찰 goal 취소 → MOVING_TO_EVENT(Nav2 NavigateToPose = 장애물 자동 회피)
  → 마커 30cm 앞 도착 → (event_type != HANDOVER 라서) PAUSED(대기)
  → 서버 '순찰재개' 버튼(set_mode RESUME) → 저장된 wp부터 순찰 복귀.

★ 이 노드는 "눈+방아쇠"일 뿐, 주행/회피/정지/대기를 직접 하지 않는다.
★ 도킹용 aruco_dock_detector(ID42)와 별개 노드로 돌린다. 서로 간섭 없음.

대역폭: raw가 아니라 compressed(JPEG) 구독 → WiFi 부담 적음(철칙 유지).
OpenCV 4.6 구 API 기준(PC·Pi 둘 다 4.6.0). 신 API 섞지 말 것.

의존 전제:
  - TF: map → 카메라 광학프레임 이 살아 있어야 함(= AMCL localize + 로봇 브링업).
    AMCL이 틀어져 있으면(문제①) map 목표도 틀어짐 → ①과 연결됨.
  - patrol_commander 가 실행 중이고 상태가 PATROLLING/ARRIVED 여야 파견 수락됨.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros
# PoseStamped 를 buffer.transform() 으로 변환하려면 이 import 가 등록되어 있어야 함
import tf2_geometry_msgs  # noqa: F401

from teamproject_interfaces.srv import DispatchToEvent

# ★3D 단계서보(2026-07-04 아이디어C): 도착 후 마커법선 기준 정면 정밀정렬
#   같은 폴더의 marker_servo.py (이 스크립트를 python3 <경로>로 실행하면 자동 임포트됨)
from marker_servo import StagedServo, ServoParams, MarkerObs, DONE as SV_DONE, FAIL as SV_FAIL


def rotmat_to_quat(R):
    """3x3 회전행렬 → (x,y,z,w). scipy 없이 직접(aruco_dock_detector 와 동일)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return x, y, z, w


def quat_zaxis_xy(qx, qy, qz, qw):
    """쿼터니언의 회전행렬 3번째 열(=로컬 +Z축)을 map 좌표에서 구해 XY만 반환.
    마커 프레임의 +Z 는 마커 면 바깥(=로봇/카메라 쪽)을 향한다."""
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    return nx, ny


class Id2FollowDetector(Node):
    def __init__(self):
        super().__init__('id2_follow_detector')

        # ── 파라미터 ──
        self.declare_parameter('image_topic', '/robot1/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/robot1/camera/camera_info')
        self.declare_parameter('marker_id', 2)             # ★추종 대상 = ID2
        self.declare_parameter('marker_length', 0.12)      # ★7/6 실측 12cm(15 가정시 거리 25% 과대→벽뒤 마커·바라보기 어긋남, 실주행 실증)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')  # 로봇 기준프레임(B1 직선용)
        self.declare_parameter('event_type', 'MARKER2')    # non-HANDOVER → 도착 시 PAUSED(대기)
        # ★네임스페이스: patrol_commander가 /robot1 네임스페이스라 서비스는 /robot1/dispatch_to_event
        self.declare_parameter('dispatch_service', '/robot1/dispatch_to_event')
        self.declare_parameter('stop_distance', 0.60)      # ★7/4: 0.30→0.60. 존 모서리 협곡 회피 — 열린 곳에 목표가 잡히게(시선은 yaw정렬이 담당)
        self.declare_parameter('min_range', 0.25)          # ★7/6: 0.5는 근접마커 기각→긴급상황 무시(실주행). 모자란 거리는 주행부가 후진으로 확보
        self.declare_parameter('max_range', 3.0)           # 이보다 멀면 pose 노이즈 커서 무시
        self.declare_parameter('confirm_count', 4)         # N프레임 연속 검출돼야 파견(오검출 방지)
        self.declare_parameter('cooldown_sec', 30.0)       # 파견 후 재파견 금지 시간
        self.declare_parameter('detect_rate', 8.0)         # Hz 상한(스로틀)
        self.declare_parameter('optical_frame', '')        # 비면 camera_info frame_id 사용
        # ★1점보정(2026-07-04): 1.0m 마커를 0.528m로 읽던 것 보정(factor 1.892).
        #   camera_info 없어서 hfov로 fx추정 → 실측 화각 ~34°(60° 아님).
        self.declare_parameter('approx_hfov_deg', 33.93)
        # ── ★3D 단계서보(아이디어C) 파라미터 ──
        #   Nav2 도착(PAUSED) 후 마커법선 기준 '수직 정면 stop_distance' 로 정밀 마무리.
        # ★런타임 마스터 스위치(7/4): 주행 중에도 켰다/껐다 —
        #   ros2 param set /id2_follow_detector enabled false   (끄기: 검출/파견/정렬 전부 정지)
        #   ros2 param set /id2_follow_detector enabled true    (켜기)
        self.declare_parameter('enabled', True)
        self.declare_parameter('use_servo', True)          # 게이트: False면 옛동작(Nav2 도착으로 끝)
        # ★yaw메모리 정렬(7/4 사용자 아이디어, 화각한계 근본해법): 파견 시 마커 map좌표를 기억,
        #   도착(PAUSED) 후 마커 안 보여도 TF(AMCL) 기준 '기억한 마커를 향하는 각'으로 제자리 회전만.
        #   회전뿐이라 금지존 안전. True면 StagedServo 대신 이걸 사용(마커 재관측 불필요).
        self.declare_parameter('use_yaw_mem', True)
        self.declare_parameter('yaw_mem_tol_deg', 2.0)   # ★7/6: 1.0은 과정밀(바라보기 용도엔 2도면 충분, 오버슛 방지)
        self.declare_parameter('yaw_mem_speed', 0.35)      # ★7/6: 0.15는 마찰로 실회전 ~4.5°/s—138° 케이스 타임아웃(실주행). 0.35로 상향
        self.declare_parameter('yaw_mem_timeout', 30.0)    # ★7/6: 반대방향(~180°) 도착 케이스 커버
        self.declare_parameter('yaw_mem_settle_ticks', 6)  # 정지 후 연속 N틱 tol 안(도킹 v5.2와 동일 설계)
        self.declare_parameter('servo_standoff', 0.45)     # 접근점 거리(법선상). 30cm보다 여유(마커가 FOV에 다 들어옴)
        self.declare_parameter('servo_max_range', 1.5)     # 서보 중 마커 수용 최대거리
        self.declare_parameter('servo_total_timeout', 45.0)  # 서보 전체 제한(초)
        self.declare_parameter('state_topic', '/robot1/state')  # patrol FSM 상태("STATE|사유|wp")
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')     # ★TwistStamped(철칙)
        # ── ★금지존(keepout) 가드 ──
        #   (a) 파견목표가 금지존이면 로봇쪽으로 당겨 재배치, (b) 서보 이동점도 검사(cmd_vel 직발행 방어)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('keepout_cost_block', 55)   # ★7/4: 90→55. RPP 코너컷 웨징 대책 — inflation 링(92~99)+소프트링1(60)도 금지 취급, 넉넉한 셀만 목표/경로 인정
        self.declare_parameter('target_pullback_step', 0.05)  # 풀백 간격(m)
        self.declare_parameter('target_pullback_max', 0.40)   # 최대 풀백(m). 넘으면 파견 포기

        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_len = float(self.get_parameter('marker_length').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.event_type = self.get_parameter('event_type').value
        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.confirm_need = int(self.get_parameter('confirm_count').value)
        self.cooldown = float(self.get_parameter('cooldown_sec').value)
        self.detect_rate = float(self.get_parameter('detect_rate').value)
        self.optical_frame_override = self.get_parameter('optical_frame').value

        # ── ArUco (4.6 구 API) ──
        dict_name = self.get_parameter('dictionary').value
        self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        s = self.marker_len / 2.0
        self.obj_pts = np.array([
            [-s,  s, 0.0], [s,  s, 0.0], [s, -s, 0.0], [-s, -s, 0.0],
        ], dtype=np.float32)

        # ── 카메라 내부파라미터 ──
        self.K = None
        self.D = None
        self.info_frame = 'camera'

        # ── TF ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 파견 서비스 클라이언트 ──
        self.dispatch_cli = self.create_client(
            DispatchToEvent, self.get_parameter('dispatch_service').value)

        # ── 검출/디바운스 상태 ──
        self.min_period = 1.0 / self.detect_rate if self.detect_rate > 0 else 0.0
        self.last_stamp = None
        self.confirm_cnt = 0          # 연속 검출 카운트
        self.last_dispatch_time = None  # 마지막 파견 시각(쿨다운)
        self.pending = False          # 파견 요청 in-flight

        # ── ★서보 상태 ──
        self.use_servo = bool(self.get_parameter('use_servo').value)
        self.servo_armed = False      # 파견 수락됨 → 도착(PAUSED) 기다리는 중
        self.servo_active = False     # 서보 주행 중(cmd_vel 발행)
        self._saw_moving = False      # MOVING_TO_EVENT 목격(그냥 PAUSED와 구분)
        self._arm_deadline = None     # arm 유효기한
        self._servo_t0 = None         # 서보 시작시각(전체 타임아웃)
        self._servo_obs = None        # 최신 MarkerObs(미소비)
        self._zero_ticks = 0          # 종료 시 0속도 지속발행 카운트
        self.patrol_state = ''        # /robot1/state 의 STATE 부분
        # ★yaw메모리 상태
        self._marker_map_xy = None    # 파견 시 기억한 마커 map좌표
        self.yawtrim_active = False   # yaw 정렬 회전 중
        self._yawtrim_t0 = None
        self._yawtrim_settle = 0
        self.servo = StagedServo(
            ServoParams(standoff=float(self.get_parameter('servo_standoff').value),
                        stop_dist=self.stop_dist),
            log=lambda s: self.get_logger().info(f'[서보] {s}'),
            point_check=self._servo_point_ok)   # ★금지존 가드 훅
        self.cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        # ★global costmap(keepout 포함) 구독 — latched
        self._costmap = None
        cm_qos = QoSProfile(depth=1,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('costmap_topic').value,
                                 self._on_costmap, cm_qos)
        self.create_subscription(String, self.get_parameter('state_topic').value,
                                 self.on_state, 10)
        self.create_timer(0.05, self._servo_loop)   # 20Hz 서보 루프(비활성 시 no-op)

        # ── I/O ──
        self.create_subscription(CameraInfo, self.info_topic,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.image_topic,
                                 self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"id2_follow_detector 시작: img='{self.image_topic}' → "
            f"dispatch_to_event, marker id={self.marker_id} len={self.marker_len}m "
            f"stop={self.stop_dist}m range=[{self.min_range},{self.max_range}]m "
            f"confirm={self.confirm_need} cooldown={self.cooldown}s")

    # ---------------------------------------------------------------
    def on_camera_info(self, msg: CameraInfo):
        if msg.k[0] <= 0.0 or msg.width == 0:
            if msg.header.frame_id:
                self.info_frame = msg.header.frame_id
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        if msg.header.frame_id:
            self.info_frame = msg.header.frame_id

    def build_approx_K(self, w, h):
        hfov = math.radians(self.get_parameter('approx_hfov_deg').value)
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        self.K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]],
                          dtype=np.float64)
        self.D = np.zeros((5, 1), dtype=np.float64)
        self.get_logger().warn(
            "근사K 사용중(camera_info 빈값) — 거리값 부정확, 캘리브 권장", once=True)

    # ---------------------------------------------------------------
    def in_cooldown(self):
        if self.last_dispatch_time is None:
            return False
        dt = (self.get_clock().now() - self.last_dispatch_time).nanoseconds / 1e9
        return dt < self.cooldown

    def on_image(self, msg: CompressedImage):
        # ★마스터 스위치: OFF면 아무것도 안 함(진행 중 정렬이 있으면 안전정지)
        if not bool(self.get_parameter('enabled').value):
            if self.servo_active or self.yawtrim_active:
                self.get_logger().warn('enabled=false — 진행 중 정렬 중단')
                self._servo_stop()
            self.confirm_cnt = 0
            return
        # ★서보 주행 중엔 파견로직 대신 근거리 관측만 공급(쿨다운/min_range 게이트 무시)
        if self.servo_active:
            self._servo_detect(msg)
            return
        if self.yawtrim_active:   # yaw정렬 중엔 검출/파견 로직 정지(회전 중 오파견 방지)
            return
        if self.pending or self.in_cooldown():
            return
        # ★7/4: 순찰 관련 상태에서만 파견 시도 — PAUSED(이벤트 대기) 등에서 같은 마커
        #   재파견 스팸(무한 거부 루프) 방지. 빈 상태('')는 상태토픽 미수신 초기라 허용.
        if self.patrol_state not in ('', 'PATROLLING', 'ARRIVED'):
            return

        # 스로틀
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and (stamp - self.last_stamp) < self.min_period:
            return
        self.last_stamp = stamp

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        if self.K is None:
            h, w = gray.shape[:2]
            self.build_approx_K(w, h)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            self.confirm_cnt = 0
            return
        ids = ids.flatten()
        idxs = np.where(ids == self.marker_id)[0]
        if len(idxs) == 0:
            self.confirm_cnt = 0
            return

        img_pts = corners[idxs[0]].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.confirm_cnt = 0
            return

        # 거리 게이팅(카메라광학 z = 전방거리)
        rng = float(tvec[2][0])
        if not (self.min_range <= rng <= self.max_range):
            self.confirm_cnt = 0
            return

        # N프레임 연속 검출 확인(오검출/순간드롭 방지)
        self.confirm_cnt += 1
        if self.confirm_cnt < self.confirm_need:
            return

        # 마커 pose(카메라 광학프레임) 구성
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotmat_to_quat(R)
        pose_cam = PoseStamped()
        # ★TF timing: 이미지 스탬프는 AMCL(map→odom) TF보다 앞서 '미래 외삽' 에러를 냄.
        #   stamp=0 → tf2가 "최신 가용 TF"로 변환(느린 순찰속도라 오차 무시가능).
        pose_cam.header.stamp = rclpy.time.Time().to_msg()
        pose_cam.header.frame_id = self.optical_frame_override or self.info_frame
        pose_cam.pose.position.x = float(tvec[0][0])
        pose_cam.pose.position.y = float(tvec[1][0])
        pose_cam.pose.position.z = float(tvec[2][0])
        pose_cam.pose.orientation.x = qx
        pose_cam.pose.orientation.y = qy
        pose_cam.pose.orientation.z = qz
        pose_cam.pose.orientation.w = qw

        # map 좌표로 변환 (TF 없으면 이번 프레임 포기, 카운트 유지)
        try:
            pose_map = self.tf_buffer.transform(
                pose_cam, self.map_frame, timeout=Duration(seconds=0.2))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f"TF 변환 실패({self.map_frame}←{pose_cam.header.frame_id}): {e}",
                throttle_duration_sec=5.0)
            return

        # 로봇 현재 위치(map) — B1: 로봇→마커 직선 계산용
        try:
            rt = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            robot_xy = (rt.transform.translation.x, rt.transform.translation.y)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"로봇 TF 조회 실패: {e}", throttle_duration_sec=5.0)
            return

        target = self.compute_stop_pose(pose_map, robot_xy)
        if target is None:
            return

        # ★yaw메모리: 파견 순간의 마커 map좌표 기억(도착 후 마커 안 보여도 시선정렬용)
        self._marker_map_xy = (pose_map.pose.position.x, pose_map.pose.position.y)
        self.send_dispatch(target, rng)

    # ---------------------------------------------------------------
    # ★금지존(keepout) 가드
    # ---------------------------------------------------------------
    def _on_costmap(self, msg: OccupancyGrid):
        self._costmap = msg

    def _cost_at_map(self, x, y):
        """map좌표 셀 cost(0~100, -1=unknown). costmap 없으면 None."""
        cm = self._costmap
        if cm is None:
            return None
        info = cm.info
        px = int((x - info.origin.position.x) / info.resolution)
        py = int((y - info.origin.position.y) / info.resolution)
        if px < 0 or py < 0 or px >= info.width or py >= info.height:
            return None
        return cm.data[py * info.width + px]

    def _map_blocked(self, x, y):
        """금지(cost≥block) 여부. 데이터 없으면 '허용'(가드는 best-effort, 서보는 저속/근거리)."""
        c = self._cost_at_map(x, y)
        if c is None:
            return False
        return c >= int(self.get_parameter('keepout_cost_block').value)

    def _reachable(self, from_xy, tx, ty):
        """★③′: costmap 자유셀(<block) BFS로 from→target 도달 가능 여부.
        costmap 없으면 True(best-effort). 52x52 규모라 BFS 비용 무시가능."""
        cm = self._costmap
        if cm is None:
            return True
        info = cm.info
        blk = int(self.get_parameter('keepout_cost_block').value)
        def cell(x, y):
            px = int((x - info.origin.position.x) / info.resolution)
            py = int((y - info.origin.position.y) / info.resolution)
            return (px, py)
        s = cell(*from_xy); t = cell(tx, ty)
        W, H = info.width, info.height
        def ok(c):
            px, py = c
            if px < 0 or py < 0 or px >= W or py >= H:
                return False
            v = cm.data[py * W + px]
            return v < blk   # unknown(-1)도 통과(best-effort)
        if not ok(t):
            return False
        if not ok(s):
            s_ok = None   # 시작셀이 lethal(낑김 등)이어도 주변 자유셀에서 출발
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    c = (s[0] + dx, s[1] + dy)
                    if ok(c):
                        s_ok = c; break
                if s_ok: break
            if s_ok is None:
                return True   # 판단불가 → 막지 않음
            s = s_ok
        from collections import deque
        q = deque([s]); seen = {s}
        while q:
            c = q.popleft()
            if c == t:
                return True
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nc = (c[0]+dx, c[1]+dy)
                if nc not in seen and ok(nc):
                    seen.add(nc); q.append(nc)
        return False

    def _servo_point_ok(self, bx, by):
        """서보 이동점(base 상대) 허용 여부 — base→map 변환 후 costmap 검사."""
        try:
            rt = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  TF 없으면 허용(저속·근거리라 리스크 작음)
            return True
        t = rt.transform.translation
        q = rt.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        mx = t.x + bx * math.cos(yaw) - by * math.sin(yaw)
        my = t.y + bx * math.sin(yaw) + by * math.cos(yaw)
        blocked = self._map_blocked(mx, my)
        if blocked:
            self.get_logger().warn(
                f'[서보] 이동점 map({mx:.2f},{my:.2f}) 금지구역 — 이동 차단',
                throttle_duration_sec=2.0)
        return not blocked

    # ---------------------------------------------------------------
    # ★3D 단계서보 (아이디어C)
    # ---------------------------------------------------------------
    def _servo_detect(self, msg: CompressedImage):
        """서보용 근거리 관측: 마커 pose를 base 프레임으로 변환해 서보에 공급."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        if self.K is None:
            h, w = gray.shape[:2]
            self.build_approx_K(w, h)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return
        ids = ids.flatten()
        idxs = np.where(ids == self.marker_id)[0]
        if len(idxs) == 0:
            return
        img_pts = corners[idxs[0]].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        rng = float(np.linalg.norm(tvec))
        if rng > float(self.get_parameter('servo_max_range').value):
            return
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotmat_to_quat(R)
        pose_cam = PoseStamped()
        pose_cam.header.stamp = rclpy.time.Time().to_msg()   # 최신 가용 TF
        pose_cam.header.frame_id = self.optical_frame_override or self.info_frame
        pose_cam.pose.position.x = float(tvec[0][0])
        pose_cam.pose.position.y = float(tvec[1][0])
        pose_cam.pose.position.z = float(tvec[2][0])
        pose_cam.pose.orientation.x = qx
        pose_cam.pose.orientation.y = qy
        pose_cam.pose.orientation.z = qz
        pose_cam.pose.orientation.w = qw
        try:  # ★base 프레임(로봇 상대) — AMCL 무관, 정적TF만 사용
            pose_base = self.tf_buffer.transform(
                pose_cam, self.base_frame, timeout=Duration(seconds=0.2))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[서보] TF(base) 실패: {e}', throttle_duration_sec=5.0)
            return
        q = pose_base.pose.orientation
        nx, ny = quat_zaxis_xy(q.x, q.y, q.z, q.w)   # 마커 +Z(면 바깥) → base XY
        self._servo_obs = MarkerObs(pose_base.pose.position.x,
                                    pose_base.pose.position.y, nx, ny)

    def on_state(self, msg: String):
        """patrol FSM 상태 추적: 파견수락→MOVING_TO_EVENT→PAUSED 에서 서보 시작."""
        # ★상태 포맷 = 'STATE|사유|번호' (파이프 1개! dock_detector_manager와 동일 파싱)
        st = (msg.data or '').split('|')[0].strip()
        prev = self.patrol_state
        self.patrol_state = st
        if not self.use_servo:
            return
        now = self.get_clock().now()
        # arm 만료
        if self.servo_armed and self._arm_deadline is not None and now > self._arm_deadline:
            self.servo_armed = False
            self._saw_moving = False
        if self.servo_armed and st == 'MOVING_TO_EVENT':
            self._saw_moving = True   # (정보용 — 판정에 필수 아님)
            # ★arm 연장(7/4 실주행 교훈): 금지존 낑김 등으로 이동이 오래 걸려도
            #   아직 MOVING_TO_EVENT면 stale이 아님 — 기한을 계속 밀어줌.
            self._arm_deadline = now + Duration(seconds=180.0)
        # 도착 판정: armed + PAUSED + 사유에 우리 event_type(예: 'PAUSED|EVENT_MARKER2|7').
        #   ★MOVING 목격을 요구하면 상태토픽 타이밍에 따라 놓침(7/4 실주행) → 사유필드로 확정.
        arrived_ours = (st == 'PAUSED' and self.event_type in (msg.data or ''))
        if not bool(self.get_parameter('enabled').value):
            return   # ★마스터 스위치 OFF — 도착해도 정렬 시작 안 함
        if self.servo_armed and arrived_ours and not self.servo_active and not self.yawtrim_active:
            self.servo_armed = False
            self._saw_moving = False
            # ★yaw메모리 모드: 마커 재관측 불필요(화각한계 무관) — 기억좌표로 제자리 시선정렬
            if bool(self.get_parameter('use_yaw_mem').value) and self._marker_map_xy is not None:
                self.yawtrim_active = True
                self._yawtrim_t0 = now
                self._yawtrim_settle = 0
                self.get_logger().info(
                    f'[yaw정렬] Nav2 도착(PAUSED) → 기억한 마커 '
                    f'map({self._marker_map_xy[0]:.2f},{self._marker_map_xy[1]:.2f}) 향해 제자리 회전')
            else:
                self.servo.reset()
                self._servo_obs = None
                self._servo_t0 = now
                self.servo_active = True
                self.get_logger().info('[서보] Nav2 도착(PAUSED) → 3D 정밀정렬 시작')
        # PAUSED 이탈(RESUME/비상 등) → 서보/yaw정렬 즉시 중단
        if (self.servo_active or self.yawtrim_active) and st != 'PAUSED':
            self.get_logger().warn(f'[서보] 상태 {prev}→{st} — 정렬 중단')
            self._servo_stop()

    def _send_cmd(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _servo_stop(self):
        self.servo_active = False
        self.yawtrim_active = False
        self._zero_ticks = 10   # 0속도 10틱(0.5s) 지속발행 → 런어웨이 방지

    def _yawtrim_tick(self):
        """★yaw메모리 정렬: TF(map→base)로 현재 위치·yaw 조회 → 기억한 마커를 향하는 각으로 회전.
        도착 xy가 어긋나도 '현 위치에서 마커를 보는 각'을 매 틱 재계산하므로 시선 정확."""
        g = lambda n: self.get_parameter(n).value
        el = (self.get_clock().now() - self._yawtrim_t0).nanoseconds / 1e9
        if el > float(g('yaw_mem_timeout')):
            self.get_logger().warn('[yaw정렬] 타임아웃 — 현 자세 유지')
            self._servo_stop()
            return
        try:
            rt = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[yaw정렬] TF 실패: {e}', throttle_duration_sec=2.0)
            self._send_cmd(0.0, 0.0)
            return
        t = rt.transform.translation
        q = rt.transform.rotation
        cur_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        mx, my = self._marker_map_xy
        target = math.atan2(my - t.y, mx - t.x)
        yerr = math.atan2(math.sin(target - cur_yaw), math.cos(target - cur_yaw))
        if abs(yerr) < math.radians(float(g('yaw_mem_tol_deg'))):
            self._send_cmd(0.0, 0.0)
            self._yawtrim_settle += 1
            if self._yawtrim_settle >= int(g('yaw_mem_settle_ticks')):
                self.get_logger().info(
                    f'[yaw정렬] ★완료 (잔여 {math.degrees(yerr):+.2f}°) — PAUSED 유지')
                self._servo_stop()
            return
        self._yawtrim_settle = 0
        self._send_cmd(0.0, math.copysign(float(g('yaw_mem_speed')), yerr))
        self.get_logger().info(f'[yaw정렬] 잔여 {math.degrees(yerr):+.1f}°',
                               throttle_duration_sec=0.5)

    def _servo_loop(self):
        # 종료 후 0속도 플러시
        if self._zero_ticks > 0:
            self._send_cmd(0.0, 0.0)
            self._zero_ticks -= 1
            return
        if self.yawtrim_active:
            self._yawtrim_tick()
            return
        if not self.servo_active:
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        # 전체 타임아웃
        el = (self.get_clock().now() - self._servo_t0).nanoseconds / 1e9
        if el > float(self.get_parameter('servo_total_timeout').value):
            self.get_logger().warn('[서보] 전체 타임아웃 — 현위치 유지(정지)')
            self._servo_stop()
            return
        obs = self._servo_obs
        self._servo_obs = None          # 1회 소비
        v, w = self.servo.tick(now_s, obs)
        if self.servo.state == SV_DONE:
            self.get_logger().info('[서보] ★정밀정렬 완료 — 마커 수직정면 도달, PAUSED 유지')
            self._servo_stop()
            return
        if self.servo.state == SV_FAIL:
            self.get_logger().warn(f'[서보] 실패({self.servo.fail_reason}) — 정지(Nav2 도착위치 유지)')
            self._servo_stop()
            return
        self._send_cmd(v, w)

    # ---------------------------------------------------------------
    def compute_stop_pose(self, pose_map: PoseStamped, robot_xy):
        """B1: 로봇→마커 직선상, 마커에서 로봇쪽으로 stop_dist 지점 + 마커를 정면으로 바라봄.
        마커 방향추정(법선)에 의존하지 않아 원거리/rotate-180 노이즈에 강함."""
        mx = pose_map.pose.position.x
        my = pose_map.pose.position.y
        dx = mx - robot_xy[0]
        dy = my - robot_xy[1]
        d = math.hypot(dx, dy)
        if d < 1e-3:
            return None
        ux, uy = dx / d, dy / d          # 로봇→마커 단위벡터
        tx = mx - self.stop_dist * ux    # 마커에서 로봇쪽으로 stop_dist(30cm)
        ty = my - self.stop_dist * uy
        yaw = math.atan2(dy, dx)         # 마커를 정면으로 바라봄

        # ★금지존 가드(a): 목표가 금지구역이면 로봇쪽으로 step씩 당겨 자유셀에 재배치.
        #   max까지 당겨도 막히면 파견 포기(금지존 안 마커에 억지로 안 감).
        step = float(self.get_parameter('target_pullback_step').value)
        pmax = float(self.get_parameter('target_pullback_max').value)
        pulled = 0.0
        while self._map_blocked(tx, ty) and pulled < pmax:
            pulled += step
            tx -= step * ux
            ty -= step * uy
        if self._map_blocked(tx, ty):
            # ★7/6: 긴급상황 무포기 원칙 — 풀백 한계여도 파견은 강행(최대접근 지점).
            #   실제 안전정지는 patrol_commander 서보의 금지존 가드(전방 셀 cost 감시)가 담당,
            #   도착 후 FACEYAW+yaw정렬로 마커 방향 응시. (구버전: 여기서 포기 → 이벤트 무시됐음)
            self.get_logger().warn(
                f'목표 주변 {pmax:.2f}m 금지구역 — 그래도 최대접근 지점({tx:.2f},{ty:.2f})으로 '
                f'파견 강행(안전정지=주행부 가드)', throttle_duration_sec=5.0)
        # ★③′도달가능 풀백(7/4 "이벤트엔 무조건 간다"): 목표셀이 자유라도 로봇 위치에서
        #   자유셀 경로(BFS)가 없으면 Nav2가 못 감 → 도달 가능해질 때까지 로봇쪽으로 당김.
        #   = '갈 수 있는 가장 가까운 지점'까지는 반드시 파견(거부 아님).
        pulled2 = 0.0
        while not self._reachable(robot_xy, tx, ty) and pulled + pulled2 < pmax:
            pulled2 += step
            tx -= step * ux
            ty -= step * uy
        if pulled2 > 0.0 and not self._reachable(robot_xy, tx, ty):
            self.get_logger().warn('도달가능 풀백 한계 — 마지막 지점으로 파견(최대접근)')
        if pulled + pulled2 > 0.0:
            self.get_logger().info(
                f'금지존 회피: 목표를 로봇쪽 {(pulled+pulled2)*100:.0f}cm 풀백 → ({tx:.2f},{ty:.2f})')

        tgt = PoseStamped()
        tgt.header.frame_id = self.map_frame
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.pose.position.x = tx
        tgt.pose.position.y = ty
        tgt.pose.position.z = 0.0
        tgt.pose.orientation.z = math.sin(yaw / 2.0)
        tgt.pose.orientation.w = math.cos(yaw / 2.0)
        return tgt

    def send_dispatch(self, target: PoseStamped, rng: float):
        if not self.dispatch_cli.service_is_ready():
            self.get_logger().warn("dispatch_to_event 서비스 미준비 — patrol_commander 확인",
                                   throttle_duration_sec=5.0)
            return
        req = DispatchToEvent.Request()
        req.target = target
        req.event_type = self.event_type
        req.target_wp_index = -1
        self.pending = True
        self.get_logger().info(
            f"ID2 발견(거리≈{rng:.2f}m) → 파견 요청: "
            f"목표 map({target.pose.position.x:.2f},{target.pose.position.y:.2f})")
        self.dispatch_cli.call_async(req).add_done_callback(self._on_dispatch_resp)

    def _on_dispatch_resp(self, future):
        self.pending = False
        self.confirm_cnt = 0
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"파견 응답 실패: {e}")
            return
        if resp.accepted:
            self.last_dispatch_time = self.get_clock().now()   # 쿨다운 시작
            self.get_logger().info(f"파견 수락: {resp.message}")
            if self.use_servo:   # ★도착(PAUSED) 감지되면 3D 서보로 마무리
                self.servo_armed = True
                self._saw_moving = False
                self._arm_deadline = self.get_clock().now() + Duration(seconds=180.0)
                self.get_logger().info('[서보] arm — Nav2 도착 대기(최대 180s)')
        else:
            # 거부(순찰중 아님 등) → 짧게만 쉬었다 재시도되게 쿨다운 안 검
            self.get_logger().info(f"파견 거부: {resp.message} (재시도 대기)")


def main():
    rclpy.init()
    node = Id2FollowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
