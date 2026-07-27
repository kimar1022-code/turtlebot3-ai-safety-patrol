#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
서버 YOLO 검출 → map 좌표 융합 → 파견 노드 (PC에서 실행, 골격)
================================================================
서버가 로봇 카메라(UDP 스트림)에서 YOLO로 화재(FIRE)/쓰러짐(FALL)을 검출해
`/robotN/safety/detections`(std_msgs/String, JSON)로 발행하면,
이 노드가 **bbox 방위각 + 라이다 거리**를 융합해 대상의 map 좌표를 만들고
patrol_commander 의 `dispatch_to_event` 서비스를 1회 호출한다.

그 뒤 흐름은 검증된 FSM이 그대로 처리한다(id2_follow_detector와 동일 설계):
  순찰 goal 취소 → MOVING_TO_EVENT(서보 접근, 0.6m 정지)
  → FACEYAW 응시 → PAUSED(대기) → 서버 RESUME 으로 순찰 복귀.

★ 이 노드는 "귀+방아쇠"일 뿐, 주행/회피/정지를 직접 하지 않는다.
★ SCHEMA_NOTE: 서버 스키마 v1.0 확정 반영 완료(2026-07-07).
  확정 payload 형태는 _parse_detections() 위 주석 참조. 파서 자체는
  '관대하게'(별칭 폴백) 유지 — 서버가 키를 바꿔도 한동안 버팀.

좌표 계산 원리:
  1) bbox 중심 픽셀 cx → 방위각 bearing = (0.5 - cx/width) * hfov  (좌+, rad)
     (카메라 광축 = 로봇 전방 가정. 카메라가 틀어져 있으면 bearing_offset_deg로 보정)
  2) /scan 에서 bearing ± sector 안의 유효 최솟값 = 거리 r
  3) TF map→base 로 로봇 (x,y,yaw) → 대상 map 좌표 = 로봇 + r*(cosθ, sinθ),
     θ = yaw + bearing.  target pose 의 yaw = θ (도착 후 FACEYAW 응시용).

의존 전제:
  - TF: map → base_footprint 살아 있어야 함(= AMCL localize + 브링업).
  - patrol_commander 실행 중 + 상태 PATROLLING 이어야 파견.
"""

import json
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       QoSDurabilityPolicy, QoSReliabilityPolicy)

from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import tf2_ros

from teamproject_interfaces.srv import DispatchToEvent


# ─────────────────────────────────────────────────────────────
# SCHEMA_NOTE ★서버 스키마 v1.0 확정(2026-07-07) 반영 완료★
#   서버 클래스 확정 4종: "fire" | "fall" | "head" | "no_helmet"
#   매핑(★서버 정본 7/8): fire→FIRE, fall/fallen_worker/fall_detected→FALL,
#         head/no_helmet/helmet→NO_HELMET (안전모 미착용, 파견 대상 →
#         enable_helmet_dispatch 기본 True. 운영 중 끄려면 False)
#   그 외(flame/person_down 등)는 관대 파싱용 별칭 — 유지해도 무방.
#   (DispatchToEvent.srv 의 event_type 은 "NO_HELMET"|"FALL"|"FIRE"|"HANDOVER")
# ─────────────────────────────────────────────────────────────
CLASS_MAP = {
    'fire': 'FIRE',
    'flame': 'FIRE',
    'fall': 'FALL',
    'fallen': 'FALL',
    'fallen_worker': 'FALL',
    'fall_detected': 'FALL',
    'person_down': 'FALL',
    # ★7/8(사용자 지정): 안전모 '미착용'만 이벤트. 'helmet'(착용)은 이벤트 아님 → 제거.
    #   서버 YOLO는 미착용을 'head' 와 'no_helmet' 둘 다(별칭)로 내보냄 → 둘 다 NO_HELMET 유지.
    'head': 'NO_HELMET',       # 미착용 (서버 별칭)
    'no_helmet': 'NO_HELMET',  # 미착용
}

# ★클래스 우선순위(서버 정책 확정 7/7): 한 메시지에 게이트(신뢰도 등)를 통과한
#   후보가 여러 클래스면 이 순서로 '1개만' 처리 — 연속프레임 스트릭 카운트와
#   파견 대상 선정 모두 이 우선순위 기준. 같은 클래스 안에선 confidence 높은 것.
EVENT_PRIORITY = ('FIRE', 'FALL', 'NO_HELMET')   # ★서버 정본(7/8): FIRE>FALL>NO_HELMET


class SafetyDispatchFusion(Node):
    def __init__(self):
        super().__init__('safety_dispatch_fusion')

        # ── 파라미터 (스키마 미확정 대비 전부 파라미터화) ──
        self.declare_parameter('robot_id', 1)              # 토픽/서비스명 조립용
        # 아래 3개는 빈 문자열이면 robot_id 로 자동 조립
        self.declare_parameter('detections_topic', '')     # 기본 /robot{N}/safety/detections
        self.declare_parameter('dispatch_service', '')     # 기본 /robot{N}/dispatch_to_event
        self.declare_parameter('state_topic', '')          # 기본 /robot{N}/state
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # 카메라 기하 — ★확정 스키마에 image_size 없음 → 이 파라미터가 정식 경로.
        #   (payload 에 image_size 가 '생기면' 그 값 우선 — 향후 추가 대비 로직 유지)
        # ★CALIB(7/9): 체커보드 캘리브 실측 fx=1268.30 → 유효 hfov=28.91°(중심부 기울기 정합).
        #   기존 62.2°(IMX219 풀센서 스펙)는 640×480 크롭 모드에서 방위각을 2.2배 부풀림
        #   → 엉뚱한 파견좌표의 근본원인. (id2 실측 33.93° 전례와 일치. 캘리브: ~/team_ws/calib/ost.yaml)
        self.declare_parameter('hfov_deg', 28.91)
        self.declare_parameter('frame_width', 640)   # ★7/7 실측: 서버 검출좌표=640스케일(방위 -49.6° 증거)
        self.declare_parameter('frame_height', 480)  # ★7/7: Pi 송신기 640x480 실송신
        # ★CALIB(7/9): 광학중심 cx=362.3px(중앙 320 대비 +42px) → +1.91° 보정(좌+).
        #   이 편차가 '도킹 살짝 왼쪽 + 사진 안 가운데'의 실체(캘리브로 수치화).
        self.declare_parameter('bearing_offset_deg', 1.91)
        # ★BEARING_MEDIAN(7/10): 중앙정렬용 bearing 을 최근 N 프레임 중앙값으로 안정화.
        #   1 = 옛 동작(한 프레임). 5 ≈ 0.5s(10fps) 지연 — 정렬 폐루프에 무해.
        self.declare_parameter('bearing_median_n', 5)
        # ★BEARING_WINDOW(7/17): 중앙값 버퍼에 '최근 N개'만이 아니라 '최근 W초 이내'만 남긴다.
        #   7/17 실측 사고: 버퍼가 생성자에서 한 번 만들어진 뒤 유실·재획득·이벤트 전환 어디서도
        #   비워지지 않아, 재획득 직후(로봇이 회전한 뒤)에도 회전 전 옛 bearing 이 중앙값을 지배했다.
        #   → 대상이 cx=115.5(화면 왼쪽 끝, ≈-9°)인데 중앙값은 -0.22°를 발행 → patrol 이
        #     "중앙정렬 완료(+1.4°)"로 오판하고 촬영(같은 순간 REFINE 은 +8.7°로 진실을 봄).
        #   검출 스트림 실측 9.75Hz → 0.5s 창 ≈ 4.9프레임 = 기존 5개와 사실상 동일(정상 시 무변화).
        #   0 = 창 비활성(옛 동작: 시간 무관 최근 N개).
        self.declare_parameter('bearing_median_window_sec', 0.5)

        # 게이팅/디바운스
        self.declare_parameter('confidence_min', 0.6)
        # ★서버 정책 확정 7/7: head/no_helmet 도 파견 대상 → 기본 True.
        #   (False 로 내리면 헬멧류 무시 — 운영 중 끄는 게이트로 유지)
        self.declare_parameter('enable_helmet_dispatch', True)
        self.declare_parameter('consecutive_frames', 2)    # ★7/16 사용자: 3→1 (단일 프레임 검출로 즉시 파견, 오탐 위험↑)
        # ★7/7: 30→60. 서버 쿨다운 60s(오탐 자동 RESUME 후 감지 억제)와 이중 정합
        self.declare_parameter('dispatch_cooldown', 60.0)  # 클래스별 재파견 금지(초)

        # 라이다 융합
        self.declare_parameter('lidar_sector_halfwidth_deg', 4.0)  # bearing ± 이 각도 안에서 최솟값
        self.declare_parameter('max_range', 4.0)           # 이보다 멀면 파견 보류(노이즈)
        # ★NEAR_EVENT(7/10): 0.3 → 0.15. 사용자 요구: "가까워서 무시"는 금지 —
        #   발견하면 무조건 파견하고, 인식 가능한 크기가 되도록 뒤로 물러나 거리를 맞춘다.
        #   0.3이면 30cm 이내 대상이 '라이다 유효거리 없음'으로 파견 자체에서 버려졌다(전체 로그 33건).
        #   후진 경로는 이미 있다: r_eff = r - stop_standoff 가 음수 → 목표가 로봇 뒤 → BACKOFF.
        #   하한 근거: robot_radius 0.1m, 라이다 range_min 0.05m → 0.15 미만은 자기몸/노이즈.
        self.declare_parameter('min_range', 0.15)          # 이보다 가까우면 보류(자기몸/오검출)
        # ★STANDOFF(7/8): 목표를 대상에서 이 거리[m]만큼 앞에 찍어 "대상 60cm 앞 정지"를 강제.
        #   미적용 시 목표가 대상 위치(라이다 풀거리)라 코앞(~15cm)까지 접근→검출유실/프레이밍불가.
        #   대상이 이보다 가까우면 목표가 로봇 뒤에 찍혀 → 기존 BACKOFF(후진)로 60cm까지 물러남.
        # ★RAW_TARGET(7/10): True=대상 원위치를 보냄(standoff는 patrol이 적용). 서버 파견과 규약 통일.
        self.declare_parameter('send_raw_target', True)
        self.declare_parameter('stop_standoff', 0.6)       # ★7/9: 1.0→0.6 원복. 대상 앞 정지거리[m] — 사용자 요구 60cm 확정(7/8의 1.0m 실험은 기각). 0=대상 위치=구동작

        # ★위치 기반 쿨다운(블랙리스트): 서버 '오탐 자동 RESUME' 신설로 생긴
        #   RESUME→재검출→재파견 무한 루프 방지. 파견 수락된 target 반경 안은
        #   클래스 무관 일정 시간 파견 금지(클래스별 쿨다운과 별개로 둘 다 적용).
        self.declare_parameter('blacklist_radius', 1.0)     # m
        self.declare_parameter('blacklist_duration', 120.0) # s
        # ★FIRE_EXEMPT(7/10, 사용자 정책): 이 클래스들은 위치 블랙리스트를 적용하지 않는다.
        #   화재는 최우선 위험이라 '이미 가본 자리'로 묶어두면 안 된다. 재파견 폭주는 쿨다운 60s가 막는다.
        #   빈 문자열('')로 두면 전 클래스에 블랙리스트 적용(옛 동작).
        self.declare_parameter('blacklist_exempt_classes', 'FIRE')

        # ★REFINE(7/7): 접근 중 목표 재보정 — MOVING_TO_EVENT 동안 같은 클래스가 화면
        #   정면(refine_bearing_max_deg 이내)에 재관측되면 그 시점 융합 좌표로 dispatch
        #   재호출 → patrol이 목표 갱신(TARGET_REFINED). 회전/화각 가장자리 첫 좌표의
        #   오차를 접근하면서 자가 교정. 쿨다운·스트릭·블랙리스트 게이트는 안 거침(갱신이므로).
        self.declare_parameter('enable_refine', True)
        self.declare_parameter('refine_bearing_max_deg', 15.0)
        self.declare_parameter('refine_min_interval', 1.0)   # 갱신 최소 간격(초)
        # ★ARRIVE_SIZE: 접근 중 bbox 세로가 화면의 이 비율 이상이면 "충분히 근접"으로
        #   보고 현위치 정지. 0.0 = 기능 끔.
        # ★7/8 기본 OFF(0.0): 큰 대상(불 포스터/배너)은 멀어도(1.5m) bbox 53%라 조기 오정지(137cm) 실증.
        #   대신 stop_standoff(60cm 거리 기반)로 과접근 방지 — 크기 무관하게 60cm 정지.
        self.declare_parameter('arrive_size_frac', 0.0)

        # 런타임 마스터 스위치 (id2 와 동일: ros2 param set ... enabled false)
        self.declare_parameter('enabled', True)

        rid = int(self.get_parameter('robot_id').value)
        self.detections_topic = (self.get_parameter('detections_topic').value
                                 or f'/robot{rid}/safety/detections')
        self.dispatch_service = (self.get_parameter('dispatch_service').value
                                 or f'/robot{rid}/dispatch_to_event')
        self.state_topic = (self.get_parameter('state_topic').value
                            or f'/robot{rid}/state')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ── TF ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 파견 서비스 클라이언트 ──
        self.dispatch_cli = self.create_client(DispatchToEvent, self.dispatch_service)

        # ── ★VISUAL_CENTER(7/8): 접근/정렬 중 활성 이벤트 대상의 실시간 방위각[rad, 좌+] 발행.
        #   patrol 이 도착 후 이 값으로 폐루프 중앙정렬(bearing→0)한다. ±15° 게이트·throttle 무관
        #   매 프레임 발행(검출 있을 때만). 미검출이면 발행 안 함 → patrol이 staleness로 유실 판정.
        self.bearing_pub = self.create_publisher(Float32, f'/robot{rid}/event/bearing', 10)

        # ── 상태 ──
        self._scan = None               # 최신 LaserScan
        self.patrol_state = ''          # /robotN/state 의 STATE 부분
        self._streak = {}               # 클래스별 연속 검출 프레임 수
        self._last_dispatch = {}        # 클래스별 마지막 파견 시각(ns) → 쿨다운
        self._cooldown_of = {}          # 클래스별 실제 적용 쿨다운(거부 시 절반)
        self.pending = False            # 파견 요청 in-flight
        self._pending_cls = None
        self._pending_target = None     # in-flight 요청의 target (x, y)
        self._blacklist = []            # [(x, y, 만료시각ns)] 위치 기반 파견 금지
        self._active_event = None       # ★REFINE: 수락된 파견의 클래스(재보정 세션)
        self._bearing_hist = []         # ★BEARING_MEDIAN(7/10)+WINDOW(7/17): [(수신시각s, bearing)] 중앙값 버퍼
        self._seen_mte = False          # ★REFINE: MOVING_TO_EVENT 진입 목격 여부(레이스 방지)
        self._last_refine_ns = 0        # ★REFINE: 마지막 갱신 시각
        self._last_refine_target = None # ★REFINE: 마지막 갱신 좌표(블랙리스트 갱신용)

        # ── 구독 ──
        # state 토픽 QoS: patrol_commander 가 transient_local RELIABLE depth1 로 발행
        # (id2_follow_detector/dock_detector_manager 와 동일 파싱 'STATE|사유|wp')
        st_qos = QoSProfile(depth=1,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self.state_topic, self._on_state, st_qos)
        # ★MISSION_TYPE(7/13 실주행 실증): patrol 이 방송하는 '현재 임무 타입' 구독.
        #   NO_HELMET 임무 중 fusion 이 눈에 띈 FALL 을 채택해 REFINE/bearing 을 엉뚱한
        #   대상으로 쏜 사고(goal 오염→금지존 모서리 3연속 물림, 탐색은 눈뜬장님) 방지.
        self._mission_event = None
        self.create_subscription(
            String, self.state_topic.rsplit('/', 1)[0] + '/event/active_type',
            self._on_mission_type, st_qos)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, self.detections_topic,
                                 self._on_detections, 10)

        self.get_logger().info(
            f"safety_dispatch_fusion 시작: det='{self.detections_topic}' "
            f"scan='{self.get_parameter('scan_topic').value}' → '{self.dispatch_service}', "
            f"hfov={self.get_parameter('hfov_deg').value}° "
            f"conf≥{self.get_parameter('confidence_min').value} "
            f"연속={int(self.get_parameter('consecutive_frames').value)}프레임 "
            f"쿨다운={self.get_parameter('dispatch_cooldown').value}s "
            f"r=[{self.get_parameter('min_range').value},{self.get_parameter('max_range').value}]m")

    # ─────────────────────────────────────────────────────────
    # 구독 콜백
    # ─────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_mission_type(self, msg: String):
        """★MISSION_TYPE(7/13): patrol 의 현재 임무 타입('FIRE'|'FALL'|'NO_HELMET'|''=없음)."""
        self._mission_event = (msg.data or '').strip() or None

    def _on_state(self, msg: String):
        # 상태 포맷 = 'STATE|사유|번호' (파이프 구분, 첫 필드만 사용)
        self.patrol_state = (msg.data or '').split('|')[0].strip()
        # ★REFINE 세션 수명: 파견 수락 직후엔 아직 PATROLLING 메시지가 남아있을 수 있어
        #   MOVING_TO_EVENT 를 한 번 본 뒤에 다른 상태가 오면 세션 종료(레이스 방지).
        if self.patrol_state == 'MOVING_TO_EVENT':
            self._seen_mte = True
        elif self._seen_mte:
            self._active_event = None
            self._seen_mte = False

    def _on_detections(self, msg: String):
        if not bool(self.get_parameter('enabled').value):
            self._streak.clear()
            return
        # ★REFINE: 접근 중이면 신규 파견 대신 목표 재보정만 시도
        if self.patrol_state == 'MOVING_TO_EVENT':
            # ★GLOBALCAM(7/8): 글로벌캠/외부 파견도 로봇캠과 동일하게 standoff 60cm+중앙정렬 적용.
            #   서버 파견은 _active_event가 없음(우리가 파견 안 했으므로) → 접근 중 최우선 검출 클래스를
            #   채택해 REFINE(standoff 재보정)+bearing(센터링) 발동. (로봇캠 파견은 이미 _active_event 있어 무영향)
            # ★MISSION_TYPE(7/13): patrol 이 임무 타입을 알려주면 그 클래스만 추적한다.
            #   (종전 '먼저 보인 클래스 채택'은 NO_HELMET 임무 중 FALL 을 물어 goal 오염 실증)
            if self._mission_event is not None:
                if self._active_event != self._mission_event:
                    self._active_event = self._mission_event
                    self.get_logger().info(
                        f'임무 타입 고정: {self._mission_event} (patrol 방송) — 다른 클래스 무시')
            elif self._active_event is None:
                et = self._pick_active_class(msg.data)
                if et is not None:
                    self._active_event = et
                    self.get_logger().info(
                        f'외부(글로벌캠) 파견 접근 중 {et} 관측 — 채택: standoff 60cm+중앙정렬 적용')
            if self._active_event is not None:
                # ★VISUAL_CENTER(7/8): 폐루프 중앙정렬용 실시간 방위각 발행(±15°/throttle 무관, 매 프레임)
                self._publish_event_bearing(msg.data)
                if bool(self.get_parameter('enable_refine').value):
                    self._try_refine(msg.data)
            return
        # ★게이트: 순찰 중(PATROLLING)일 때만 파견 시도. PAUSED/도킹중 등 무시.
        if self.patrol_state != 'PATROLLING':
            self._streak.clear()
            return
        if self.pending:
            return

        parsed = self._parse_detections(msg.data)
        if parsed is None:
            return
        dets, stamp = parsed

        # ── 신뢰도 게이트 + 클래스별 최고 신뢰도 후보 ──
        conf_min = float(self.get_parameter('confidence_min').value)
        best = {}   # event_type → 이번 프레임 최고 신뢰도 검출
        for d in dets:
            if d['confidence'] < conf_min:
                continue
            et = d['event_type']
            if et not in best or d['confidence'] > best[et]['confidence']:
                best[et] = d

        # ── ★우선순위 선택(서버 정책 7/7): FIRE > FALL > HELMET 중 1개만 처리.
        #    스트릭도 선택된 클래스만 +1, 나머지는 전부 리셋 — 화재가 보이는 동안
        #    쓰러짐/헬멧 스트릭은 쌓이지 않음(항상 상위 이벤트 우선) ──
        sel = None
        for et in EVENT_PRIORITY:
            if et in best:
                sel = et
                break
        for et in list(self._streak.keys()):
            if et != sel:
                self._streak[et] = 0
        if sel is None:
            return
        self._streak[sel] = self._streak.get(sel, 0) + 1

        # ── 연속 프레임 디바운스 + 쿨다운 → 파견(한 번에 1건, in-flight 중복 방지) ──
        need = int(self.get_parameter('consecutive_frames').value)
        if self._streak[sel] < need:
            return
        if self._in_cooldown(sel):
            # ★7/9: 무음 차단 금지 — "감지되는데 왜 안 가?" 규명용 가시화
            self.get_logger().info(f'{sel}: 쿨다운 중 — 파견 보류',
                                   throttle_duration_sec=10.0)
            return
        self._try_dispatch(sel, best[sel], stamp)

    # ─────────────────────────────────────────────────────────
    # payload 파서 — SCHEMA_NOTE ★서버 스키마 v1.0 확정(2026-07-07)★
    # ─────────────────────────────────────────────────────────
    # 확정 payload:
    #   {"schema_version": "1.0",
    #    "created_at": "2026-07-07T05:07:30.123Z",  # ISO-8601 UTC(Z 접미)
    #                       # ⚠️촬영시각 아닌 '페이로드 생성시각'(추론 후, 오차 있음)
    #    "frame_stamp": 1783400850.1,               # ★촬영시각 float 초(서버에 요청중,
    #                       #   오면 created_at 보다 1순위로 사용 — _pick_stamp 참조)
    #    "camera_id": "robot1",                     # robot_id 일치 검증(멀티로봇 오수신 방지)
    #    "source_topic": "/robot1/live/image",      # (미사용)
    #    "detections": [
    #       {"class": "fire",                       # "fire"|"fall"|"head"|"no_helmet"
    #        "confidence": 0.852,
    #        "bbox_xyxy": [120, 200, 320, 450],
    #        "center_px": [220.0, 325.0],
    #        "face_result": null}]}                 # (미사용)
    #   image_size 필드 없음 확정 → frame_width/frame_height 파라미터가 정식 경로.
    #   아래 별칭 폴백(class_name/label, conf/score, stamp 등)은 관대 파싱용으로 유지.
    # ─────────────────────────────────────────────────────────
    def _parse_detections(self, raw: str):
        try:
            data = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn('detections JSON 파싱 실패 — 서버 payload 확인',
                                   throttle_duration_sec=10.0)
            return None
        if isinstance(data, list):          # 최상위가 리스트인 스키마도 수용
            top, det_list = {}, data
        elif isinstance(data, dict):
            top = data
            det_list = data.get('detections', data.get('objects', []))
        else:
            return None
        if not isinstance(det_list, list):
            return None

        # ★camera_id ↔ robot_id 일치 검증(확정 스키마): 멀티로봇에서 남의 검출
        #   오수신 방지. 부재 시 통과(관대).
        cam_id = top.get('camera_id')
        if cam_id is not None:
            expect = f"robot{int(self.get_parameter('robot_id').value)}"
            if str(cam_id).strip().lower() != expect:
                self.get_logger().warn(
                    f"camera_id '{cam_id}' ≠ '{expect}' — 남의 검출 스킵"
                    f"(detections_topic/서버 라우팅 확인)", throttle_duration_sec=10.0)
                return None

        top_stamp = self._pick_stamp(top)
        out = []
        for d in det_list:
            if not isinstance(d, dict):
                continue
            # 클래스명 (여러 후보 키)
            cls = None
            for k in ('class', 'class_name', 'label', 'name'):
                if k in d:
                    cls = str(d[k]).strip().lower()
                    break
            if cls is None:
                self.get_logger().warn('검출에 class 필드 없음 — 스킵(서버 스키마 확인)',
                                       throttle_duration_sec=10.0)
                continue
            et = CLASS_MAP.get(cls)
            if et is None:      # 매핑에 없는 클래스 → 조용히 무시
                continue
            # ★헬멧 게이트: 기본 True(서버 정책 확정 7/7) — False 로 내리면 무시
            if et == 'NO_HELMET' and not bool(
                    self.get_parameter('enable_helmet_dispatch').value):
                continue
            # 신뢰도
            conf = None
            for k in ('confidence', 'conf', 'score'):
                if k in d:
                    try:
                        conf = float(d[k])
                    except (TypeError, ValueError):
                        pass
                    break
            if conf is None:
                self.get_logger().warn('검출에 confidence 없음 — 스킵',
                                       throttle_duration_sec=10.0)
                continue
            # 이미지 크기: 검출 개별 > 최상위 > 파라미터
            img_sz = self._pick_image_size(d) or self._pick_image_size(top)
            if img_sz is None:
                img_sz = (int(self.get_parameter('frame_width').value),
                          int(self.get_parameter('frame_height').value))
            # bbox 중심 픽셀
            cx = self._pick_center_x(d)
            if cx is None:
                self.get_logger().warn('검출에 center_px/bbox 없음 — 스킵',
                                       throttle_duration_sec=10.0)
                continue
            out.append({'event_type': et, 'confidence': conf,
                        'cx': cx, 'width': float(img_sz[0]),
                        'size_frac': self._pick_size_frac(d, img_sz),  # ★ARRIVE_SIZE: bbox 세로/화면 비율
                        'stamp': self._pick_stamp(d) or top_stamp})
        # 메시지 대표 stamp: 검출 개별 stamp > 최상위 stamp > None(수신 시각 처리)
        return out, top_stamp

    @staticmethod
    def _pick_size_frac(d, img_sz):
        """★ARRIVE_SIZE: bbox 세로 높이 / 화면 세로 비율(0~1). bbox 없으면 None."""
        for k in ('bbox_xyxy', 'bbox', 'xyxy'):
            b = d.get(k)
            if isinstance(b, (list, tuple)) and len(b) == 4:
                try:
                    h = abs(float(b[3]) - float(b[1]))
                    return max(0.0, min(1.0, h / float(img_sz[1])))
                except (TypeError, ValueError, ZeroDivisionError):
                    return None
        return None

    @staticmethod
    def _pick_stamp(d):
        """stamp 후보 추출 → epoch float 초 or None.
        ★우선순위(7/7): frame_stamp(float 초) > created_at(ISO-8601 Z) > 별칭들.
          - frame_stamp = '촬영 시각' (서버에 추가 요청중 — 오면 자동으로 1순위 사용)
          - created_at  = 페이로드 '생성시각'(YOLO 추론 후) — 지연만큼 방위각 오차 있음
        float 초 / {sec,nanosec} dict / ISO 문자열 전부 관대하게 수용.
        파싱 실패/부재 시 None → 호출측이 수신시각(latest TF)으로 폴백."""
        for k in ('frame_stamp', 'created_at', 'stamp', 'timestamp', 'time'):
            v = d.get(k)
            if v is None:
                continue
            if isinstance(v, str):          # ISO-8601 (확정 스키마 경로)
                try:
                    return datetime.fromisoformat(
                        v.replace('Z', '+00:00')).timestamp()
                except ValueError:
                    continue
            if isinstance(v, dict):
                try:
                    return float(v.get('sec', 0)) + float(v.get('nanosec', 0)) * 1e-9
                except (TypeError, ValueError):
                    continue
            try:
                return float(v)
            except (TypeError, ValueError):
                continue
        return None

    @staticmethod
    def _pick_image_size(d):
        v = d.get('image_size') or d.get('img_size')
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            try:
                return (float(v[0]), float(v[1]))
            except (TypeError, ValueError):
                return None
        w, h = d.get('width'), d.get('height')
        if w and h:
            try:
                return (float(w), float(h))
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _pick_center_x(d):
        """중심 x픽셀: center_px 우선, 없으면 bbox 에서 계산."""
        c = d.get('center_px') or d.get('center')
        if isinstance(c, (list, tuple)) and len(c) >= 1:
            try:
                return float(c[0])
            except (TypeError, ValueError):
                pass
        for k in ('bbox_xyxy', 'bbox', 'xyxy'):
            b = d.get(k)
            if isinstance(b, (list, tuple)) and len(b) >= 4:
                try:
                    return (float(b[0]) + float(b[2])) / 2.0
                except (TypeError, ValueError):
                    continue
        return None

    # ─────────────────────────────────────────────────────────
    # 융합: 방위각 + 라이다 거리 → map 좌표
    # ─────────────────────────────────────────────────────────
    def _bearing_from_cx(self, cx, width):
        """bbox 중심 픽셀 → 방위각(rad, 좌+). 카메라 광축=로봇 전방 가정."""
        hfov = math.radians(float(self.get_parameter('hfov_deg').value))
        off = math.radians(float(self.get_parameter('bearing_offset_deg').value))
        return (0.5 - cx / width) * hfov + off

    def _lidar_range_at(self, bearing):
        """/scan 에서 bearing ± sector 안의 유효 최솟값. 없으면 None."""
        scan = self._scan
        if scan is None:
            self.get_logger().warn('scan 미수신 — 거리 융합 불가, 파견 보류',
                                   throttle_duration_sec=5.0)
            return None
        half = math.radians(float(self.get_parameter('lidar_sector_halfwidth_deg').value))
        rmin = max(float(self.get_parameter('min_range').value), scan.range_min)
        rmax = min(float(self.get_parameter('max_range').value), scan.range_max)
        best = None
        ang = scan.angle_min
        for r in scan.ranges:
            # 각도 랩어라운드 대비: bearing 과의 차를 [-π,π]로 정규화해 비교
            diff = math.atan2(math.sin(ang - bearing), math.cos(ang - bearing))
            ang += scan.angle_increment
            if abs(diff) > half:
                continue
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            if best is None or r < best:
                best = r
        return best

    def _robot_pose_at(self, stamp):
        """map→base TF 로 로봇 (x, y, yaw). stamp(float초) 기준, 실패 시 latest 폴백.
        ★stamp 기준을 먼저 시도하는 이유: 서버 왕복 지연 동안 로봇이 회전하면
          '지금 자세' 기준 방위각이 어긋남 → 검출 시각의 자세로 계산해야 정확."""
        tries = []
        if stamp is not None:
            tries.append(rclpy.time.Time(seconds=stamp))
        tries.append(rclpy.time.Time())   # latest
        for t in tries:
            try:
                rt = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, t,
                    timeout=Duration(seconds=0.2))
                tr = rt.transform.translation
                q = rt.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return tr.x, tr.y, yaw
            except Exception as e:  # noqa: BLE001
                last_err = e
        self.get_logger().warn(f'TF({self.map_frame}←{self.base_frame}) 실패: {last_err}',
                               throttle_duration_sec=5.0)
        return None

    # ─────────────────────────────────────────────────────────
    # 파견
    # ─────────────────────────────────────────────────────────
    def _bl_exempt(self, et):
        """★FIRE_EXEMPT(7/10): 위치 블랙리스트를 적용하지 않는 클래스(사용자 정책).
        화재는 최우선 위험 — '이미 가본 자리'라고 두 번째 발화를 무시하면 안 된다.
        재파견 폭주는 쿨다운(60s)이 계속 막는다(블랙리스트만 면제, 쿨다운은 유지)."""
        raw = str(self.get_parameter('blacklist_exempt_classes').value)
        return et in {s.strip().upper() for s in raw.split(',') if s.strip()}

    def _blacklisted(self, x, y, et):
        """★위치 블랙리스트: '같은 클래스'의 수락된 파견 target 반경 안이면 True(만료 항목 정리).
        서버 오탐 자동 RESUME → 같은 자리 재파견 무한 루프 방지.
        ★7/9 픽스: 클래스별 분리 — FALL 블랙리스트가 FIRE 파견까지 막던 문제(좁은 맵에서
        반경 1m 몇 개면 전역이 덮여 '화재 계속 감지되는데 파견 0건' 실주행 실증).
        ★7/10: FIRE는 면제(_bl_exempt). 7/9 실주행에서 같은 불이 0.89m 떨어진 좌표로 융합돼
        반경 1.0m 안에 걸려 120s간 무시됐다(사용자: '불을 계속 인식했는데 안 멈추고 감')."""
        if self._bl_exempt(et):
            return False
        now = self.get_clock().now().nanoseconds
        self._blacklist = [b for b in self._blacklist if b[2] > now]
        rad = float(self.get_parameter('blacklist_radius').value)
        for bx, by, _exp, bet in self._blacklist:
            if bet == et and math.hypot(x - bx, y - by) <= rad:
                return True
        return False

    def _in_cooldown(self, et):
        t0 = self._last_dispatch.get(et)
        if t0 is None:
            return False
        dt = (self.get_clock().now().nanoseconds - t0) / 1e9
        return dt < self._cooldown_of.get(
            et, float(self.get_parameter('dispatch_cooldown').value))

    def _try_dispatch(self, et, det, msg_stamp):
        stamp = det.get('stamp') or msg_stamp   # 검출 개별 > 메시지 대표 > None(=latest)
        bearing = self._bearing_from_cx(det['cx'], det['width'])
        r = self._lidar_range_at(bearing)
        if r is None:
            self.get_logger().info(
                f'{et}: 방위각 {math.degrees(bearing):+.1f}° 라이다 유효거리 없음'
                f'(범위 밖/가림) — 파견 보류', throttle_duration_sec=5.0)
            return
        pose = self._robot_pose_at(stamp)
        if pose is None:
            return
        rx, ry, ryaw = pose
        theta = ryaw + bearing            # map 기준 대상 방향각
        # ★RAW_TARGET(7/10): target 은 '대상 원위치'를 보낸다. standoff(60cm 당기기)는 patrol 이
        #   _apply_dispatch_standoff 한 곳에서만 적용한다.
        #   이유: 서버(글로벌캠) 파견은 대상 원위치를 보내는데 fusion 은 보정본을 보내 **의미가 달랐다**
        #   → 서버 파견은 goal 이 대상 위에 찍혀 Nav2 가 'collision ahead → abort'(7/10 실증).
        #   스키마 v1.0 에 원본/보정본 구분 필드가 없으므로 규약을 "target=대상 원위치"로 통일.
        #   send_raw_target=False 로 두면 옛 동작(여기서 standoff 적용) — 그때는 patrol 의
        #   dispatch_apply_standoff 도 False 로 내려야 이중 적용을 피한다.
        if bool(self.get_parameter('send_raw_target').value):
            r_eff = r
        else:
            r_eff = r - float(self.get_parameter('stop_standoff').value)
        tx = rx + r_eff * math.cos(theta)
        ty = ry + r_eff * math.sin(theta)

        # ★위치 블랙리스트 검사(★7/9: 같은 클래스만, 클래스별 쿨다운과 별개)
        if self._blacklisted(tx, ty, et):
            self.get_logger().info(
                f'{et}: 목표 map({tx:.2f},{ty:.2f}) 블랙리스트 반경 내 — 파견 금지'
                f'(오탐 재파견 루프 방지)', throttle_duration_sec=10.0)
            return

        target = PoseStamped()
        target.header.frame_id = self.map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = tx
        target.pose.position.y = ty
        # yaw = θ : 로봇이 대상을 바라보는 방향(도착 후 FACEYAW 응시용)
        target.pose.orientation.z = math.sin(theta / 2.0)
        target.pose.orientation.w = math.cos(theta / 2.0)

        if not self.dispatch_cli.service_is_ready():
            self.get_logger().warn('dispatch_to_event 서비스 미준비 — patrol_commander 확인',
                                   throttle_duration_sec=5.0)
            return

        req = DispatchToEvent.Request()
        req.target = target
        req.event_type = et
        req.target_wp_index = -1
        self.pending = True
        self._pending_cls = et
        self._pending_target = (tx, ty)
        self.get_logger().info(
            f'★{et} 검출(conf={det["confidence"]:.2f}, 방위 {math.degrees(bearing):+.1f}°, '
            f'거리 {r:.2f}m) → 파견 요청: map({tx:.2f},{ty:.2f})')
        self.dispatch_cli.call_async(req).add_done_callback(self._on_dispatch_resp)

    def _pick_active_class(self, raw):
        """★GLOBALCAM: 외부(글로벌캠) 파견 접근 중 채택할 클래스 — 검출된 것 중 최우선(EVENT_PRIORITY),
        conf 게이트 통과분. 없으면 None(아직 대상 미관측)."""
        parsed = self._parse_detections(raw)
        if parsed is None:
            return None
        dets, _ = parsed
        conf_min = float(self.get_parameter('confidence_min').value)
        present = {d['event_type'] for d in dets if d['confidence'] >= conf_min}
        for et in EVENT_PRIORITY:
            if et in present:
                return et
        return None

    def _publish_event_bearing(self, raw):
        """★VISUAL_CENTER(7/8): 활성 이벤트 클래스의 최고신뢰 검출 방위각[rad,좌+]을 매 프레임 발행.
        게이트(±15°)/throttle 없음 — patrol이 도착 후 이 값으로 폐루프 회전(bearing→0)해 중앙 정렬.
        미검출 프레임엔 발행 안 함 → patrol이 마지막 수신후 경과시간(staleness)으로 유실 판정."""
        parsed = self._parse_detections(raw)
        if parsed is None:
            return
        dets, _ = parsed
        conf_min = float(self.get_parameter('confidence_min').value)
        best = None
        for d in dets:
            if d['event_type'] != self._active_event or d['confidence'] < conf_min:
                continue
            if best is None or d['confidence'] > best['confidence']:
                best = d
        if best is None:
            return
        # ★IMG_CENTER(7/9): 중앙정렬용 bearing은 '이미지 화면 중심' 기준(광학 오프셋 제외).
        #   _bearing_from_cx는 광학중심(cx=362px) 기준이라 이 값으로 0°를 만들면 사진에선
        #   대상이 화면중심보다 +42px 오른쪽에 섬(캘리브 실측). 사진 프레이밍이 목적이므로
        #   오프셋을 빼서 bearing→0 = 대상이 픽셀 320(화면 정중앙)에 오게 한다.
        #   (파견 '좌표' 계산은 각도 정확도가 목적이라 광학 보정 유지 — 용도가 다름)
        off = math.radians(float(self.get_parameter('bearing_offset_deg').value))
        bearing = self._bearing_from_cx(best['cx'], best['width']) - off
        # ★BEARING_MEDIAN(7/10, 실주행): 큰 대상(불 포스터)은 YOLO bbox 가 프레임마다 다른 영역을
        #   잡아 cx 가 228→391→576→318 로 요동친다(실측). 한 프레임만 믿으면 '정면'이라 착각한
        #   순간에 정렬을 끝내고, 정작 사진엔 대상이 가장자리에 잡힌다(FIRE_163507).
        #   최근 N 프레임 중앙값으로 안정화 — 평균이 아니라 중앙값이라 튀는 프레임에 안 끌린다.
        n = int(self.get_parameter('bearing_median_n').value)
        if n > 1:
            # ★BEARING_WINDOW(7/17): 시간창 밖(=대상 유실·재획득·이벤트 전환 전) 표본을 먼저 버린다.
            #   버퍼를 (시각, bearing) 으로 들고 있으면 유실 훅을 따로 달지 않아도 자동으로 비워진다.
            #   재획득 직후엔 표본이 3개 미만이라 아래 median 을 건너뛰고 최신 원값이 그대로 나간다
            #   = 회전 전 옛값이 중앙값을 지배하던 오판(7/17 cx=115.5 → -0.22°) 차단.
            now_s = self.get_clock().now().nanoseconds / 1e9
            self._bearing_hist.append((now_s, bearing))
            w = float(self.get_parameter('bearing_median_window_sec').value)
            if w > 0.0:
                self._bearing_hist = [(t, b) for (t, b) in self._bearing_hist if now_s - t <= w]
            if len(self._bearing_hist) > n:
                self._bearing_hist = self._bearing_hist[-n:]
            if len(self._bearing_hist) >= 3:
                vals = sorted(b for (_t, b) in self._bearing_hist)
                bearing = vals[len(vals) // 2]
        # ★BEARING_DIAG(7/10): 사진이 안 가운데인데 로그는 '정면'이라 한 사건(FIRE 163507).
        #   실측 bbox 중심 515px(640폭) = -8.8° 인데 로그는 -2.1°. cx/width 원본을 남겨 대조한다.
        self.get_logger().info(
            f'BEARING_DIAG {best["event_type"]}: cx={best["cx"]:.1f} width={best["width"]} '
            f'conf={best["confidence"]:.2f} → bearing={math.degrees(bearing):+.2f}° '
            f'(화면중심 기준, 0이면 cx={best["width"]/2:.0f})',
            throttle_duration_sec=1.0)
        self.bearing_pub.publish(Float32(data=float(bearing)))

    def _try_refine(self, raw):
        """★REFINE(7/7): 접근 중 재보정. 같은 클래스가 화면 정면에 재관측되면
        그 시점 융합 좌표로 dispatch 재호출(=patrol이 TARGET_REFINED 로 목표 갱신)."""
        now = self.get_clock().now().nanoseconds
        if now - self._last_refine_ns < float(self.get_parameter('refine_min_interval').value) * 1e9:
            return
        parsed = self._parse_detections(raw)
        if parsed is None:
            return
        dets, stamp = parsed
        conf_min = float(self.get_parameter('confidence_min').value)
        best = None
        for d in dets:
            if d['event_type'] != self._active_event or d['confidence'] < conf_min:
                continue
            if best is None or d['confidence'] > best['confidence']:
                best = d
        if best is None:
            return
        bearing = self._bearing_from_cx(best['cx'], best['width'])
        if abs(math.degrees(bearing)) > float(self.get_parameter('refine_bearing_max_deg').value):
            return   # 아직 정면 아님 — 첫 좌표로 계속 접근
        pose = self._robot_pose_at(best.get('stamp') or stamp)
        if pose is None:
            return
        rx, ry, ryaw = pose
        theta = ryaw + bearing
        # ★ARRIVE_SIZE: bbox가 충분히 크면(=대상이 화면을 채우면) 더 안 다가가고 현위치 정지.
        #   목표를 현위치로 갱신 → 서보 잔여거리 0 → 도착 → FACEYAW 응시 → PAUSED(촬영).
        #   라이다 거리 무관하게 "사진 프레이밍" 기준으로 멈추는 안전장치(과접근/금지존 진입 예방).
        asf = float(self.get_parameter('arrive_size_frac').value)
        sf = best.get('size_frac')
        r = None   # ★감사픽스: ARRIVE_SIZE 분기서 미정의 방지
        if asf > 0.0 and sf is not None and sf >= asf:
            tx, ty = rx, ry
            self.get_logger().info(
                f'★ARRIVE_SIZE {self._active_event}: bbox 세로 {sf*100:.0f}% ≥ {asf*100:.0f}% '
                f'— 현위치 정지(응시 방위 {math.degrees(bearing):+.1f}°)')
        else:
            r = self._lidar_range_at(bearing)
            if r is None:
                return
            # ★RAW_TARGET(7/10): REFINE 도 초기 파견과 동일 기준 — '대상 원위치'를 보낸다.
            #   (여기만 standoff를 빼면 재보정할 때마다 목표가 60cm씩 더 뒤로 밀린다.)
            if bool(self.get_parameter('send_raw_target').value):
                r_eff = r
            else:
                r_eff = r - float(self.get_parameter('stop_standoff').value)
            tx = rx + r_eff * math.cos(theta)
            ty = ry + r_eff * math.sin(theta)
        if not self.dispatch_cli.service_is_ready():
            return
        target = PoseStamped()
        target.header.frame_id = self.map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = tx
        target.pose.position.y = ty
        target.pose.orientation.z = math.sin(theta / 2.0)
        target.pose.orientation.w = math.cos(theta / 2.0)
        req = DispatchToEvent.Request()
        req.target = target
        req.event_type = self._active_event
        req.target_wp_index = -1
        self._last_refine_ns = now
        self._last_refine_target = (tx, ty)
        # ★감사픽스(7/7): ARRIVE_SIZE 분기에선 r 미정의 — f-string이 NameError로 노드를
        #   통째로 죽이던 확정 크래시. 거리 표기를 분기 안전하게.
        dist_txt = f'{r:.2f}m' if r is not None else '크기도달(현위치 정지)'
        self.get_logger().info(
            f'★REFINE {self._active_event}: 정면 재관측(방위 {math.degrees(bearing):+.1f}°, '
            f'거리 {dist_txt}) → 목표 갱신 map({tx:.2f},{ty:.2f})')
        self.dispatch_cli.call_async(req).add_done_callback(self._on_refine_resp)

    def _on_refine_resp(self, future):
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'★REFINE 응답 실패: {e}')
            return
        if resp.accepted and self._last_refine_target is not None:
            # 갱신 좌표(진짜 위치에 수렴)도 블랙리스트 등록 — RESUME 후 재파견 루프 방지 유지
            dur = float(self.get_parameter('blacklist_duration').value)
            exp = self.get_clock().now().nanoseconds + int(dur * 1e9)
            self._blacklist.append((self._last_refine_target[0],
                                    self._last_refine_target[1], exp,
                                    self._active_event))   # ★7/9: 클래스별 분리
        elif not resp.accepted:
            self.get_logger().info(f'★REFINE 거부: {resp.message} (상태 전이 중 — 무해)')

    def _on_dispatch_resp(self, future):
        self.pending = False
        et = self._pending_cls
        tgt = self._pending_target
        self._pending_cls = None
        self._pending_target = None
        self._streak.clear()   # 파견 시도 후 연속 카운트 처음부터
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'파견 응답 실패: {e}')
            return
        cd = float(self.get_parameter('dispatch_cooldown').value)
        self._last_dispatch[et] = self.get_clock().now().nanoseconds
        if resp.accepted:
            self._cooldown_of[et] = cd
            # ★수락된 target 만 위치 블랙리스트 등록(오탐 RESUME 재파견 루프 차단)
            if tgt is not None and self._bl_exempt(et):
                self.get_logger().info(f'블랙리스트 면제({et}) — 등록 생략, 쿨다운 {cd:.0f}s만 적용')
            elif tgt is not None:
                dur = float(self.get_parameter('blacklist_duration').value)
                exp = self.get_clock().now().nanoseconds + int(dur * 1e9)
                self._blacklist.append((tgt[0], tgt[1], exp, et))
                self.get_logger().info(
                    f'블랙리스트 등록({et}): map({tgt[0]:.2f},{tgt[1]:.2f}) '
                    f'반경 {float(self.get_parameter("blacklist_radius").value):.1f}m '
                    f'{dur:.0f}s (같은 클래스만 차단)')
            self.get_logger().info(f'파견 수락({et}): {resp.message} — 쿨다운 {cd:.0f}s')
            self._active_event = et    # ★REFINE 세션 시작(접근 중 목표 재보정 허용)
            self._seen_mte = False
        else:
            # 거부(순찰중 아님 등) → 쿨다운 절반만 적용해 다음 기회 재시도 여지
            self._cooldown_of[et] = cd * 0.5
            self.get_logger().warn(
                f'파견 거부({et}): {resp.message} — 쿨다운 절반({cd*0.5:.0f}s) 후 재시도')


def main():
    rclpy.init()
    node = SafetyDispatchFusion()
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
