#!/usr/bin/env python3
"""
patrol_commander.py — FSM 순찰 제어 (16개 상태)

오늘 모델 반영:
  - 상태 16개: EVENT_PAUSED/WAITING_PLAY/MANUAL_PAUSED -> PAUSED 통합, MOVING_TO_EVENT 추가
  - IDLE 자동시작 제거 -> set_mode 서비스로 PATROL_START 대기
  - /state 발행 형식: "STATE|reason|index" 3필드 (status_reporter가 이 형식을 파싱)
  - robot_id 파라미터화 + 토픽/액션 상대경로 (namespace로 robotN 분리)
  - 파견 인터페이스 DispatchToEvent (서버 -> commander, 안전모 접근촬영 / HANDOVER)
  - STUCK 복구: stuck_retry_wait초 대기 후 stuck_max_retries회 재시도 -> 소진 시 wp 스킵
  - 명명 규칙: snake_case, 함수는 동사 시작

★ESTOP_FIX (2026-06-23) — EMERGENCY_STOP "안 멈춤" 버그 + 정지·재개 버그 수정:
  1. current_goal_handle 보관  2A. cancel_current_goal()  2B. handle_goal_result 가드
  4. MANUAL_ENTER에서 save_waypoint_index()

★BATTERY Phase 1/2/3 (2026-06-23):
  - Phase 1: /battery_state 구독 + battery_threshold. ARRIVED에서 임계 미만이면 LOW_BATTERY.
  - Phase 2: LOW_BATTERY → 충전소 좌표로 이동(RETURNING_TO_CHARGER) → 도착 시 CHARGING.
  - Phase 3: CHARGING에서 charge_target_pct(85%) 도달 OR charging_timeout 경과 시 충전완료
             → RESUMING_AFTER_CHARGE → 저장된 wp부터 순찰 재개.
             (지금은 단자를 '사람이 손으로' 꽂아 배터리를 올림. Phase 4 정밀도킹 시 자동화)

★HANDOVER (2026-07-01):
  - DispatchToEvent.event_type == 'HANDOVER' → 파견 도착 시 PAUSED 대신
    target_wp_index부터 순찰 재개(PATROLLING). 다른 로봇의 잔여 순찰 인계용.

명명 규칙 메모:
  - __init__, main 은 파이썬/ROS2 고정 진입점이라 이름 유지.
  - 콜백 매개변수는 실제 사용하는 값이라 _ 를 붙이지 않음(PEP8: _는 미사용 매개변수용).
"""
import os
import math
import json    # ★ROUTE_SERVO: route 그래프(geojson) 로드용
import heapq         # ★GRAPH_ONLY(7/10): 엣지 최단경로(거리가중 다익스트라)
import yaml
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)   # ★ROUTE_SERVO: /scan 구독용
from rclpy.time import Time as TfTime             # ★ROUTE_SERVO: TF 최신값 조회용
import tf2_ros                                    # ★ROUTE_SERVO: map→base_link 피드백

from nav2_msgs.action import NavigateToPose, Spin, DriveOnHeading   # ★ESCAPE: 탈출 복구용
from nav2_msgs.msg import CollisionMonitorState   # ★OBSTACLE_BRIDGE: 라이다 정지 신호 구독용
from nav_msgs.msg import OccupancyGrid, Odometry   # ★ESCAPE: local costmap / odom 구독
from builtin_interfaces.msg import Duration as MsgDuration   # ★ESCAPE: 액션 time_allowance
from geometry_msgs.msg import (PoseStamped, Point, Quaternion,
                               PoseWithCovarianceStamped, TwistStamped)  # ★ROUTE_SERVO: 직접 주행명령
from std_msgs.msg import String, Int32, Float32, Bool
from sensor_msgs.msg import BatteryState, LaserScan   # ★BATTERY / ★ROUTE_SERVO 전방가드
from rcl_interfaces.srv import SetParameters     # ★DOCK_TOL: wp별 도착오차 동적 변경용
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory

from teamproject_interfaces.srv import SetMode, DispatchToEvent
from teamproject_interfaces.msg import ObstacleVerdict


class PatrolState(Enum):
    IDLE = 0
    LOCALIZING = 1
    PATROLLING = 2
    ARRIVED = 3
    RETRYING = 4
    STUCK = 5
    OBSTACLE_WAITING = 6
    MOVING_TO_EVENT = 7
    PAUSED = 8                 # EVENT_PAUSED/WAITING_PLAY/MANUAL_PAUSED 통합
    RESUMING = 9
    MANUAL_CONTROL = 10
    LOW_BATTERY = 11
    RETURNING_TO_CHARGER = 12
    CHARGING = 13
    RESUMING_AFTER_CHARGE = 14
    EMERGENCY_STOP = 15
    ESCAPE = 16              # ★ESCAPE: 빨강(인플레이션/금지존)에 물렸을 때 뚫린 쪽으로 탈출 후 재계획
    DOCK_DWELL = 17          # ★DWELL: 도킹존 도착 후 N초 대기(2바퀴째부터) 뒤 순찰 재개
    DOCKING = 18             # ★DOCK_FIX: pre-dock 도착 후 후진도킹으로 도킹존(0,0) 실제 진입 대기
    UNDOCKING = 19           # ★UNDOCK: 도킹존(0,0) 빨강칸을 전진으로 빠져나온 뒤 순찰 시작(매 출발)


# (★OBSTACLE_BRIDGE 이후) verdict 타임아웃은 'obstacle_verdict_timeout' 파라미터로 이전됨.
# 아래 상수는 더 이상 사용하지 않음(참고용 기본값).
OBSTACLE_VERDICT_TIMEOUT = 8.0
RETRY_WAIT = 2.0   # RETRYING에서 재시도 전 대기(초) — abort 직후 즉시 재전송 방지

# ★ROUTE_SERVO 튜닝값 — route_servo_lap.py(7/6 실주행 2랩+랩→도킹 풀사이클 검증)와 동일값 이식.
#   값 변경 시 원본 스크립트와 함께 바꿀 것.
RS_RATE = 20.0            # 제어주기 [Hz]
RS_SECTORS = 12          # ★UNWEDGE(7/10): 라이다 30°×12섹터 — 끼임 탈출 방향 판단용
RS_TURN_KP = 1.5          # 회전 P게인
RS_TURN_MAX = 0.18        # 최대 회전속도 [rad/s] (★7/13: 0.22→0.18 "회전각·회전속도 더 줄이자" — 2Hz 검출이 회전을 따라오게. 7/10: 0.30→0.22, 7/9: 0.35→0.30)
RS_TURN_MIN = 0.10        # 최소 회전속도 (정지마찰 극복) ★7/13: 0.15→0.10 — MAX 감소에 맞춰 제어폭 유지
RS_TURN_TOL = 0.06        # 회전 완료 허용오차 [rad] (~3.4°)
                          # ★TURN_TIMEOUT_DERIVED(7/10): 하드코딩 15.0 → RS_TURN_MAX 에서 유도.
                          #   회전속도를 0.30→0.22 로 내리자 180° 회전이 10.5s→14.3s 가 되어
                          #   고정 15s 와 겨우 0.7s 차이 → 정상 상황에서도 'TURN 타임아웃' 발생(실주행).
                          #   근본원인은 '속도는 바꾸고 그 속도에 묶인 타임아웃은 안 바꾼 것'.
                          #   최악(180°) 등속시간 × 1.5 + 여유 2s. 앞으로 속도를 바꿔도 자동 추종.
RS_TURN_TIMEOUT = math.pi / RS_TURN_MAX * 1.5 + 2.0    # [s]  (0.22 → 23.4s)
RS_DRIVE_V = 0.10         # 직진 속도 [m/s] (★7/9: 0.12→0.10 사용자 요청 감속)
RS_DRIVE_V_NEAR = 0.05    # 도착 직전 속도
RS_NEAR_DIST = 0.15       # 감속 시작 거리 [m]
RS_YAW_KP = 1.2           # 직진 중 yaw 보정 게인
RS_YAW_CLAMP = 0.18       # 직진 중 최대 보정 회전 [rad/s] ★7/13: 0.3→0.18 (시각추종 중 홱 도는 조향 감속 — 사용자 요청)
RS_RETURN_ANG = 0.6       # 직진 중 이 각도[rad] 이상 틀어지면 재조준
RS_XY_TOL = 0.07          # 노드 도착 판정 [m]
RS_FRONT_STOP = 0.12      # 전방 급정지 거리 [m] — 노드 근처 전용(순찰선=벽 0.2m라 이 값이어야 코너 도착 가능)
RS_FRONT_STOP_FAR = 0.30  # ★FRONT_STOP_FAR(7/22): 구간 중간 급정지 거리 [m].
                          #   라이다가 로봇 앞면보다 ~0.19m 뒤(7/4 실측) → 0.12 에서 정지하면 이미
                          #   범퍼가 닿은 뒤다(7/22 정적장애물 테스트서 실제 박음). 노드까지 먼
                          #   구간 중간에서만 적용 — 노드 근처는 벽이 정상적으로 가까워 0.12 유지.
RS_FRONT_SLOW = 0.40      # ★DYN_OBS(7/7): 이 거리부터 계단 감속 시작(지나가는 사람/이동체 여유)
RS_FRONT_HALF_ANG = 25.0  # 전방 감시 반각 [deg]
RS_ACC_V = 0.25           # 가감속 램프 [m/s²] — 계단명령=바퀴슬립=odom오염 방지
RS_ACC_W = 1.5            # 회전 램프 [rad/s²]
RS_SETTLE_TICKS = 8       # 회전후/도착후 정지 틱(0.4s) — 멈춘 스캔으로 AMCL 보정 틈

# ★CLEAR_DETOUR: OBSTACLE_WAITING(서보 모드)에서 전방이 이 거리[m] 이상 다시 보이면 '뚫림' 후보.
#   정지 기준(RS_FRONT_STOP_FAR 0.30)보다 넉넉히 잡아 히스테리시스 확보(라이다 None=range_min
#   미만일 수 있어 뚫림으로 안 침). 7/22: 0.30→0.45 (정지점이 0.30으로 오르며 경계 겹침 해소).
CD_FRONT_CLEAR = 0.45

class PatrolCommander(Node):
    def __init__(self):
        super().__init__('patrol_commander')

        # --- 파라미터 ---
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('stuck_max_retries', 2)      # STUCK에서 재시도 횟수
        self.declare_parameter('stuck_retry_wait', 5.0)     # 재시도 전 대기(초)
        self.declare_parameter('dispatch_max_retries', 5)   # 파견 goal 실패 시 즉시 재시도 횟수
        self.declare_parameter('battery_threshold', 33.0)   # ★BATTERY: 충전 복귀 임계 %(30+3마진). 테스트는 param set으로 80 등
        self.declare_parameter('handover_battery_threshold', 40.0)  # ★HANDOVER(7/7 정책확정): 도킹 도착 시 이 %이하면 2호기 교대요청 (50→40)
        # ★SERVER_BATTERY(7/17 사용자 확정): 배터리 판단 주체 = 서버.
        #   "우린 도착하면 무조건 하도버 날려. 하도버 했는데 배터리가 없어서 못 나가는 건
        #    서버가 판단할 일이야." + "차징의 정의를 바꾸자 — 도킹존에서 배터리가 올라가면 차징."
        #   ★7/17 사용자 최종: "나가는 건 IDLE이야." + "도킹한 로봇이 왜 항상 차징이야 —
        #     배터리가 오르기 전엔 차징 아니고 아이들이야."
        #   True  = ① 도킹 도착 시 배터리 무관 **항상** handover 발행(로봇은 배터리로 판단 안 함)
        #           ② 도킹 직후 상태 = **IDLE** (배터리로 CHARGING 자물쇠를 걸지 않는다)
        #           ③ CHARGING 은 **관측 사실**일 때만 — 도크존에서 배터리가 실제로 오르면
        #              _charge_watch_tick 이 IDLE → CHARGING 으로 라벨링(CHARGE_OBS)
        #           ④ 출발은 종전대로 IDLE 에서만. 충전 중(CHARGING)인 로봇을 내보내야 하면
        #              서버가 RESET(→IDLE) 후 PATROL_START. 내보낼지 말지는 전적으로 서버 판단.
        #   False = 옛 동작(로봇이 40% 이하면 스스로 CHARGING 잠금, 아니면 IDLE)
        self.declare_parameter('use_server_battery_policy', True)
        # ★CHARGE_OBS 문턱(7/17 사용자: "자꾸 차징으로 바뀌어 — 3프로 이상 올라가면 차징으로"):
        #   배터리 리포트가 실측 ±1.1% 요동(로봇1 62.77↔63.87, 전압은 11.53→11.64V 정상 상승)해서
        #   1%(charge_progress_delta) 문턱은 노이즈만으로 넘어간다 → 전용 문턱 3%.
        #   charge_progress_delta(1%)는 충전완료/정체 감지가 쓰므로 건드리지 않는다.
        self.declare_parameter('charge_obs_delta', 3.0)
        # ★CHARGE_SWAP(7/17): CHARGING 중이라도 이 % 이상이면 교대 PATROL_START 수락.
        # ★기본 False (사용자 지적으로 철회, 7/17): "본인이 배터리가 낮아서 다른 애를 내보내는 건데
        #   35% 제한을 두는 건 말이 안 된다" — 맞는 지적이다. CHARGING = 도킹 시 40% 이하였다
        #   = '쉬어야 하는 쪽'인데, 그걸 35%에 내보내면 battery_threshold(33%)에서 곧장 충전복귀가
        #   걸려 2% 쓰고 왕복만 한다. '둘 다 CHARGING이면 순찰 정지'는 버그가 아니라 정상 —
        #   두 대 다 40% 아래면 순찰할 배터리가 없는 게 사실이고, 답은 충전(85% → IDLE → 서버 투입).
        #   남겨둔 이유: 상대 로봇이 죽어 어쩔 수 없이 내보내야 하는 예외 상황용 탈출구.
        #   켤 거면 charging_dispatch_min_pct 를 battery_threshold(33%)보다 충분히 높게 둘 것.
        self.declare_parameter('use_charging_dispatch', False)
        self.declare_parameter('charging_dispatch_min_pct', 35.0)
        # ★IDLE_BATT_GATE(7/10): IDLE 파견 수락 배터리 하한 %. battery_threshold(33)보다 위여야 한다 —
        #   같거나 낮으면 출동 직후 LOW_BATTERY로 되돌아오는 왕복이 생긴다. 0으로 두면 옛 동작(무제한).
        self.declare_parameter('dispatch_min_battery', 40.0)
        self.declare_parameter('dock_zone_radius', 0.15)   # ★DOCK_FIX: 로봇 pose가 도킹존(0,0) 이 반경(m) 이내면 '도킹존 도착' 확정
        # ★DOCK_ZONE_NODISPATCH(7/16, 사용자: "원래 도킹존에서는 객체인식 안 했다"): 도킹 접근/재시도 중
        #   이거나 로봇이 도크 이 반경(m) 안이면 파견(이벤트) 거부 — 도크존에서 이벤트에 끌려나가
        #   도킹이 무한 지연되던 것 차단(R1 도킹 재시도 중 FIRE 파견 수락 실측). 0으로 두면 옛 동작.
        self.declare_parameter('dispatch_dock_exclude_radius', 0.0)  # ★7/16 수정: 0.7→0.0 (근접만으로 막으니 순찰 중 helmet 파견까지 차단됨 → 실제 도킹재시도 _rs_dock_retry일 때만 막게)
        self.declare_parameter('dock_dwell_sec', 10.0)      # ★DWELL: 2바퀴째부터 도킹존 대기(초)
        self.declare_parameter('charge_target_pct', 85.0)   # ★BATTERY Phase3: 이 % 이상이면 충전완료
        # ★DOCK_POSE(7/14, 멀티로봇): 로봇별 도크 map 좌표 — 위치주입(DRIFT_RESET/LOC_GATE)의 기준점.
        #   로봇1=(0,0,0) 기본. 로봇2=(0, 0.24, 0): 로봇1 도크서 +y 24cm (바퀴갭 8cm 실측 기하).
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_sig_rear_max', 0.35)  # ★DOCK_SIG(7/15): 뒷벽이 이 거리 이내여야 '도크에 있음'(실측 0.18m)
        self.declare_parameter('dock_yaw', 0.0)
        # ★STAY_DOCKED(7/14, 사용자: "내가 출발 누를 때까지 안 나오게"): 충전완료 후 자동 순찰
        #   재개 금지 — IDLE 대기(재투입은 관제 PATROL_START/파견만). 실측: RTC 도킹 직후 배터리
        #   88%≥85%라 '충전완료→자동 언도크→순찰'로 계속 튀어나감. True 면 옛 동작(자동 재개).
        self.declare_parameter('auto_resume_after_charge', False)
        self.declare_parameter('charging_timeout', 600.0)   # (★CHARGE_FIX 이후 미사용 — 블라인드 강제완료 제거됨. 호환 위해 선언만 유지)
        self.declare_parameter('charge_progress_delta', 1.0)   # ★CHARGE_FIX: 이 %만큼 오르면 '충전중'으로 인정(상승 감지)
        self.declare_parameter('charge_stall_timeout', 60.0)   # ★CHARGE_FIX: 이 초 동안 delta만큼도 안 오르면 '충전 안 됨' 경고(계속 대기)
        self.declare_parameter('undock_timeout', 20.0)   # ★UNDOCK: Pi UNDOCK_DONE 안 오면 이 초 뒤 페일세이프(순찰 강행)
        self.declare_parameter('dock_timeout', 150.0)    # ★DOCK: Pi DOCK_DONE/도킹존진입 안 되면 이 초 뒤 페일세이프(STUCK). 7/6 실측: 서치+정렬+전진30+180°+후진+트림 ≈60~120s라 60은 성공도킹을 오판(실제 발생)
        self.declare_parameter('use_dock_executor', False)  # ★#3 게이트: True여야 undock/dock(Pi 실행노드) 사용. 기본 False=옛 동작(안전, 내일 순찰 무영향)
        # ★ROUTE_SERVO 게이트(7/6): True=PATROLLING 주행을 AGV식 직진 서보(route 그래프)로 대체.
        #   이벤트/도킹/배터리/ESCAPE FSM은 무손상 — 주행부만 교체. False=기존 Nav2 waypoint 순찰(waypoints.yaml).
        #   기본 True 승격(7/7): 7/6 실주행 2랩+랩→도킹 풀사이클 검증 완료. 끄기: ros2 param set /robotN/patrol_commander use_route_servo false
        self.declare_parameter('use_route_servo', True)
        # ★VISUAL_CENTER(7/8): 이벤트 도착 후 카메라 실시간 방위각으로 폐루프 중앙정렬.
        self.declare_parameter('center_tol_deg', 1.5)   # ★7/14: 2→1.5 (±2°=±44px 복권폭이 사진 편차 주범 중 하나, 사용자 요청). 7/13: 3→2    # |방위| 이 이내면 중앙 도달(정렬 완료)
        self.declare_parameter('centering_sign', 1.0)    # 회전 부호. 대상이 반대로 밀려나면 -1.0으로 뒤집기
        # ★EVENT_FINALIZE(7/9): 이벤트 도착 마무리 — 중앙정렬 후 ②라이다 실측거리 60cm 미세조정
        #   ③벽피팅 평행트림(대상 표면 정면 대향). 카메라 좌표목표의 누적오차를 센서 폐루프로 흡수.
        self.declare_parameter('use_event_lidar_finalize', True)  # 현장에서 끄기: ros2 param set ... false
        # ★DISPATCH_STANDOFF(7/10): 파견 target(=대상 원위치)에서 event_stop_range 만큼 당겨 goal 생성.
        #   서버(글로벌캠)·fusion 파견 모두 동일 규약. False = 옛 동작(받은 좌표 그대로 goal).
        self.declare_parameter('dispatch_apply_standoff', True)
        # ★RAW_GOAL(7/10, 사용자 요청): "글로벌캠이 가라는 좌표로 가되 금지존·장애물은 코스트맵으로
        #   피하고, 도착하면 회전하며 대상을 찾는다."
        #   True  = standoff(60cm) 를 빼지 않고 받은 좌표 자체를 goal 로 삼는다.
        #           단 그 좌표가 대상 위라 인플레이션에 걸리면 Nav2 가 경로를 거부하므로
        #           (실주행 전례: 'detected collision ahead → abort'), 전역 코스트맵을 보고
        #           로봇 쪽으로 5cm 씩만 당겨 '계획 가능한 가장 가까운 점'을 goal 로 쓴다.
        #           조기정지(EARLY_STOP)는 자동으로 꺼진다 — 좌표까지 가야 하므로.
        #   False = 기존 동작(대상 60cm 앞에서 정지). 안 되면 이 값을 false 로 되돌릴 것.
        #   ★7/13 기본 True 승격(사용자 확정 루틴): 글로벌캠 좌표로 직행 → 한바퀴 탐색 → 중앙정렬.
        #     실기 2회 검증(goal 오차 9.3~24cm 도달).
        self.declare_parameter('use_raw_goal_event', True)
        # ★SETTLE_NEAR(7/10): 접근 실패 시 포기하지 말고 현 위치에서 대상 응시 후 마무리(사진 확보).
        self.declare_parameter('settle_near_on_fail', True)
        self.declare_parameter('event_stop_range', 0.6)   # 대상 정지거리[m] (fusion stop_standoff와 짝)
        self.declare_parameter('event_range_tol', 0.05)   # 거리 허용오차[m] (±5cm)
        self.declare_parameter('event_drive_v', 0.06)     # ★EV_SLOW(7/15): 이벤트 접근 전용 속도상한[m/s] — 검출(2Hz)·시각추종이 따라오게. 순찰 랩은 RS_DRIVE_V 그대로
        # ★DET_SLOW(7/17, 사용자 제안): safety/detections 에 검출이 잡히는 동안 감속 접근.
        #   목적 = 검출~파견확정 사이 구멍 메우기(그동안 랩 0.10 그대로 달려 bbox 가 흔들렸다).
        #   ★부작용 인지 필요: 지나가는 사람도 검출이라 랩이 그때마다 느려진다(랩 시간↑).
        #     끄려면 use_detect_slow:=false, 덜 느리게 하려면 detect_slow_v 를 올릴 것.
        # ★UNWEDGE_LIDAR(7/17): 끼임(발자국이 금지존 안) 시 이동가드의 '금지존 검사'만 면제하고
        #   라이다는 유지 — 후진 거리는 라이다가 정한다(상수 15cm 박기는 벽 충돌 위험, 사용자 확정).
        #   False = 옛 동작(중심점 기준 _pose_in_keepout 만 인정 → 발자국 끼임 시 교착 재발).
        self.declare_parameter('use_unwedge_lidar', True)
        self.declare_parameter('use_detect_slow', True)
        self.declare_parameter('detect_slow_v', 0.06)         # 검출 중 속도상한[m/s]
        self.declare_parameter('detect_slow_hold_sec', 1.5)   # 마지막 검출 후 이 시간까지 감속 유지
        # ★7/9 밤 실주행: 35°는 틀렸다. 60cm에서 화면 폭 30cm(hfov 28.3°)라 트림으로 6~10° 넘게 돌면
        #   대상이 화각을 벗어나 bearing이 끊기고, CENTER2가 '현 방향 촬영'으로 포기한다(3회 전부).
        #   "트림 후 CENTER2가 되찾는다"는 전제 자체가 거짓 — 안 보이면 못 되찾는다.
        #   → 화각 절반(14.16°)에서 대상 크기·여유를 뺀 값으로 제한. 초과분은 스킵(비스듬한 채 촬영)하되,
        #   그래도 유실되면 TRIM_UNDO 가 트림 직전 각도로 되돌아가 재획득한다.
        # ★7/9 밤 사용자 확정: 평행트림 제거. 마무리 각도 기준은 '대상을 정면으로 본다' 하나뿐.
        #   벽과 나란히 서기(트림)는 대상 응시와 다른 목표라, 한 파이프라인에 섞으면 뒤에 오는 쪽이
        #   앞의 정렬을 파괴한다(도킹 SRV_FACE wall↔cam 교대와 같은 오류). 실주행 3회 전부
        #   트림이 대상을 28° 화각 밖으로 밀어내 CENTER2가 '현 방향 촬영'으로 포기했다.
        # ★7/13 기본 True 복귀(사용자 2순위 확정: "60cm 앞에 멈춰서 수평이 되게 자세를 맞출 것").
        #   트림이 대상을 화각 밖으로 밀던 문제는 TRIM_UNDO+CENTER2(7/9)가 되돌려 잡는다.
        self.declare_parameter('use_event_trim', True)
        self.declare_parameter('event_trim_max_deg', 12.0)   # ★7/13 최종: 화각 반각(14.5°) 이내만 — 큰 트림은 대상을 프레임 밖으로 던져 자기모순(획 돌기 실측)
        # ★NAV2_EVENT(7/8): 이벤트 접근을 Nav2 경로계획으로(금지존/장애물 회피). 도착 후 폐루프 중앙정렬 → PAUSED.
        # ★★(7/9 당시 경고) 서보 접근(False)은 코스트맵을 안 봐 금지존을 가로질렀다 → 당시엔 "절대 False 금지".
        # ★★7/13 기본 False 복귀(사용자 확정): 7/10에 _rs_cmd 한 곳에 마스크 하드가드+라이다∪코스트맵
        #   세트 게이트가 들어가 서보 경로가 전 구간 보호된다(7/9 경고의 전제 소멸). 반대로 Nav2 접근은
        #   ①하드가드 사각(cmd_vel이 _rs_cmd를 안 거침, 7/10 침범 실증) ②fusion 좌표 요동에 goal이
        #   끌려다님(7/13 실증: FALL 오염·유령좌표·벽박음) → "발견 즉시 대상을 화면 중앙에 놓고
        #   계속 보면서 달린다"(서보+REFINE 매틱 반영+VISUAL_TRACK 조향)가 사용자 확정 루틴.
        self.declare_parameter('use_nav2_event_approach', False)
        # ★HYBRID_FALLBACK(7/13 실주행): 서보 직선이 막혔을 때 —
        #   잔여거리 ≤ blocked_settle_max → 그 자리 촬영(사용자: "1m면 찍자")
        #   잔여거리 >  blocked_settle_max → 이 이벤트만 Nav2 우회로 전환(planner가 돌아가는 길을 찾음).
        #   종전 '무조건 최근접 안전점 도착 처리'는 1.2m 밖 사진 = 안면인식 불가(실측).
        self.declare_parameter('blocked_settle_max', 1.0)
        # ★VISUAL_TRACK(7/9): 서보 이벤트 접근 시에만 쓰이는 /event/bearing 폐루프 조향.
        #   ★7/13: 서보 접근이 기본이 되면서 이 조향이 "대상을 화면 중앙에 유지하며 주행"의 본체.
        #   (금지존·장애물은 _rs_cmd의 마스크 하드가드+라이다∪코스트맵 세트가 전 구간 담당)
        self.declare_parameter('use_visual_event_tracking', True)
        self.declare_parameter('route_nodes', '1,2,3,4,5,6,7,8,9,10,11,12,13')  # 랩 노드열(마지막=프리도킹 노드)
        self.declare_parameter('route_graph', '~/team_ws/maps/patrol_graph.geojson')
        # ★DIAG_BAN v2(7/13): 엣지 경로가 이 노드(충전소)를 관통할 때만 직선 직행 허용
        self.declare_parameter('route_avoid_via', '14')
        self.declare_parameter('max_laps', 3)   # ★MAX_LAPS: 이 바퀴수만큼 도킹하면 도킹존에 정지(주차, undock 안 함). 0=무한반복
        # ★OBSTACLE_BRIDGE: 라이다(collision_monitor) 기반 장애물 정지 파라미터
        self.declare_parameter('obstacle_enter_debounce', 0.6)   # STOP이 이 초 이상 지속되면 장애물 확정(순간 스침 무시)
        self.declare_parameter('obstacle_clear_debounce', 1.0)   # 전방이 이 초 이상 다시 뚫리면 치워진 것(동적 통과) → 자동 재개
        self.declare_parameter('obstacle_verdict_timeout', 8.0)  # OBSTACLE_WAITING에서 verdict 무응답 시 STUCK까지 대기(초). 서버 통합 전엔 크게 설정 권장
        # ★CLEAR_DETOUR(A안): 서보 모드 정적 장애물 → 서버 CLEAR 판정 시 Nav2 하이브리드 우회.
        #   ★7/22 기본 True 승격 — 게이트 OFF 상태로 정적장애물 테스트에서 우회 없이
        #   급정지→ESCAPE 로 빠지는 것 실측(박음+노드스킵). 끄기: use_clear_detour:=false
        self.declare_parameter('use_clear_detour', True)
        self.declare_parameter('rs_obstacle_wait_sec', 5.0)  # ★CLEAR_DETOUR: 서보 DRIVE 전방막힘이 이 초(기본 5s) 연속이면 OBSTACLE_WAITING 전이
        # ★EMERGENCY_PATROL(협의 확정, 시나리오 3-5/4-3): 순찰(PATROLLING) 중 서버 YOLO가
        #   직접 EMERGENCY verdict를 쏘면 즉시 정지 + PAUSED(EVENT_*) 촬영 대기.
        #   기본 True — 서버가 EMERGENCY를 안 쏘면 무동작이라 무해. 끄기 스위치 용도:
        #   ros2 param set /robot1/patrol_commander accept_emergency_in_patrol false
        self.declare_parameter('accept_emergency_in_patrol', True)
        # ★DOCK_TOL: wp별 도착오차 — 일반 wp는 느슨(덜 들어감), 마지막 wp(=충전존)만 정밀 도킹
        self.declare_parameter('patrol_xy_tolerance', 0.15)   # 일반 wp 도착오차(m). 2026-07-01: 0.10↔0.18 중간값 0.15 시도(0.18은 코너컷팅으로 금지존 밟았음, 0.15로 절충)
        self.declare_parameter('dock_xy_tolerance', 0.05)     # 마지막 wp(충전존) 정밀 도킹 오차(m)
        # ★DOCK_YAW: wp별 yaw 도착오차 — 일반 wp는 전역과 동일(느슨, 교착방지), 도킹존만 정밀
        self.declare_parameter('patrol_yaw_tolerance', 0.45)  # 일반 wp yaw 오차(rad, 전역과 동일)
        self.declare_parameter('dock_yaw_tolerance', 0.2443)  # ★도킹존만 14° 정밀. ⚠️AMCL yaw~18°보다 작아 교착 위험 — 교착 시 이 값 ↑
        # ★ESCAPE: 빨강(인플레이션/금지존) 탈출 복구 파라미터
        self.declare_parameter('escape_max_attempts', 3)      # 탈출 시도 횟수(소진 시 STUCK)
        self.declare_parameter('escape_distance', 0.25)       # 한 번에 전진할 거리(m)
        self.declare_parameter('escape_speed', 0.08)          # 전진 속도(m/s)
        self.declare_parameter('escape_probe_radius', 0.5)    # 방향별 뚫림 탐색 반경(m)
        self.declare_parameter('escape_num_dirs', 16)         # 방향 샘플 개수
        self.declare_parameter('escape_cost_block', 90)       # local costmap 이 값(0~100) 이상이면 막힘/빨강
        # ★KEEPOUT_SET(7/10): 이동 가드용 임계. 99=치명(254)+내접(253)만 — 금지존 마스크는 치명으로
        #   칠해지므로 이 값이면 '금지존/벽'만 걸리고 인플레이션(≤98) 오탐이 없다. 90은 너무 예민.
        self.declare_parameter('keepout_cost_block', 99)
        # ★MOVE_GUARD(7/10): 서보 병진을 라이다∪금지존으로 검사(사용자 원칙).
        #   'event'(기본) = 이벤트 접근·마무리에서만. 도킹은 항상 면제(빨강칸 진입 불가해짐),
        #                   랩 순찰은 검증된 루프라 손대지 않음(사용자 확정).
        #   'all' = 랩까지 포함(도킹은 여전히 면제). 'off' = 옛 동작.
        self.declare_parameter('move_guard_scope', 'event')
        # ★MASK_HARDGUARD(7/10, 사용자 절대원칙): 금지존 마스크는 어떤 상태에서도 밟지 않는다.
        #   코스트맵과 독립. 도킹존·순찰선은 마스크 밖이라 면제가 필요 없다(실측 확인).
        self.declare_parameter('use_mask_hardguard', True)
        self.declare_parameter('keepout_mask_yaml', '~/team_ws/maps/keepout_mask_newmap.yaml')   # ★7/10 버그픽스: 옛 맵(55x55)용 keepout_mask.yaml 을 읽고 있었다. 실사용은 newmap(52x52, map.yaml과 짝)
        self.declare_parameter('mask_guard_look', 0.25)    # 진행방향 이 거리까지 검사[m]
        self.declare_parameter('mask_guard_radius', 0.10)  # 로봇 반경(발자국 부풀리기)[m]
        # ★GOAL_SAFE(7/10): 파견 goal 은 금지존에서 이만큼 떨어져야 한다.
        #   ★7/13: 0.15→0.10 (사용자: "15cm는 너무 길어"). 반경 0.10 그대로, 여유분 제거.
        self.declare_parameter('goal_keepout_clearance', 0.10)
        # ★GOAL_NEAREST(7/17 사용자 확정): goal 이 금지존/인플레이션 안이면 '로봇 쪽으로 후퇴'가 아니라
        #   **동심원을 넓혀가며 찾은 밖 최근접점**을 목적지로 쓴다. False = 옛 동작(직선 후퇴).
        self.declare_parameter('use_goal_nearest_outside', True)
        # ★EARLY_STOP(7/10): 접근 중 대상을 직접 보고 60cm에 닿으면 goal 취소하고 그 자리서 마무리.
        #   글로벌캠/융합 좌표의 오차를 실측으로 덮어쓴다(사용자 요구).
        self.declare_parameter('use_event_early_stop', True)
        self.declare_parameter('early_stop_bearing_deg', 8.0)   # 협각 라이다(±8°)가 대상을 볼 조건
        # ★REACQUIRE(7/10): 대상 유실 시 저장 좌표를 맹신하지 않고 제자리 재탐색으로 되찾는다.
        self.declare_parameter('use_event_reacquire', True)
        # ★BEARING_FRESH(7/10): bearing 유실 판정 시간. 짧으면 스윕이 중앙정렬과 싸운다.
        self.declare_parameter('bearing_fresh_sec', 1.2)
        self.declare_parameter('reacquire_max_deg', 25.0)   # 시작 yaw 기준 ± 훑는 각도
        self.declare_parameter('reacquire_w', 0.05)         # ★7/15: 0.08→0.05 사용자 "객체 찾을 때 회전 늦춰"(검출갭 3s×0.08=14° 과회전 핑퐁). 타임아웃 자동유도
        # ★REACQ_TIMEOUT_FIX(7/10): 8.0 → 12.0. ±25°를 0.15rad/s로 1왕복하면 (25+50)/8.6°/s = 8.7s 인데
        #   타임아웃이 8.0s 라 스윕을 끝내기 전에 항상 포기했다 — 대상을 찾을 기회가 구조적으로 없었다
        #   (실주행 로그: '재탐색 타임아웃 — 대상 못 찾음'이 매번). 왕복 8.7s + 여유 3.3s.
        #   ⚠reacquire_max_deg / reacquire_w 를 바꾸면 이 값도 (3·amp/w + 여유)로 다시 잡을 것.
        self.declare_parameter('reacquire_timeout', 12.0)   # 이 시간 넘으면 옛 폴백(창 1개 기준)
        # ★SEARCH_CYCLES(7/10): 1왕복 실패 시 창을 옮겨 다시 훑는 횟수(사용자: "없으면 다시 움직이고 찾아")
        self.declare_parameter('reacquire_cycles', 3)
        # ★FULL_CIRCLE(7/13, 사용자 확정 루틴): 도착 후 창 왕복 스윕 대신 **한 방향 360° 연속 회전**으로
        #   대상을 찾는다("제자리에서 한바퀴 돌면서 이벤트를 찾는걸로"). 검출 즉시 중앙정렬로 전환.
        #   속도는 reacquire_w 공유(느릴수록 서버 YOLO 검출 기회↑, 0.10rad/s → 1바퀴 63s).
        #   False 면 옛 창 스윕(±25°×3창)으로 복귀.
        self.declare_parameter('reacquire_full_circle', True)
        # ★EVENT_SEARCH_DIR(7/16, 사용자 요청 "객체인식 탐색 회전을 반대로"): 한바퀴(full_circle)
        #   탐색의 회전 방향 부호. +1.0=CCW(좌, 기존), -1.0=CW(우, 반전). 되돌리려면 1.0.
        #   AIMED 재탐색(마지막 목격 방향 최단회전)은 대상방향이라 이 부호와 무관(영향 없음).
        self.declare_parameter('event_search_dir', -1.0)
        # ★BACKUP_REACQ(7/13, 사용자: "접근중 유실이면 뒤로"): 방금까지 보이다 유실 = 지나쳤을
        #   가능성 — 한바퀴 돌기 전에 먼저 온 방향으로 후진(가드 통과)하며 재획득을 시도한다.
        self.declare_parameter('use_backup_reacquire', True)
        self.declare_parameter('backup_reacq_dist', 0.30)    # 최대 후진 거리[m]
        self.declare_parameter('backup_reacq_window', 5.0)   # '방금까지 보였다' 판정 창[s]
        # ★PHOTO_SPOT(7/14, 사용자: "서버 좌표=이벤트서 60cm 떨어진 최적 촬영지점. 진짜 그 좌표로 가"):
        #   11:57 FIRE 실측 — 파견 2.3s 만에 SNAP_STOP 이 도착 처리해 1.5m 밖에서 촬영.
        #   ①SNAP_STOP 은 goal 잔여거리 ≤ max_remaining 일 때만(멀면 bearing 주입만 하고 계속 접근)
        #   ②SERVO_60CM 의 map거리 컷 게이트(좌표=촬영지점이라 '좌표 60cm 앞 정지'는 1.2m 이탈)
        #   ③이벤트 도착 판정 조임(0.30→0.05m. AMCL 오차 ±3~5cm 라 1cm 는 물리적 불가)
        self.declare_parameter('snap_stop_max_remaining', 99.0)  # ★7/15 저녁 사용자: "신호 한 번 받으면 무조건 멈춰" — 원거리 보류 폐지(0.5→99). 원거리컷 감수, 되돌리려면 0.5
        self.declare_parameter('use_map_range_cut', False)
        # ★7/14 사용자: "최소 2cm까지". AMCL 오차와 싸울 수 있어 DRIVE 타임아웃 시
        #   잔여<0.15m 면 도착 처리하는 안전망과 세트(2cm 헌팅 교착 방지).
        self.declare_parameter('event_arrive_tol', 0.02)
        # ★AIMED_REACQ(7/14, 사용자: "무조건 360이 아니라 보였던 곳으로, 회전각도 줄이면서"):
        #   유실 시 한바퀴(79s) 전에 '마지막 목격 map 방향'으로 최단 조준 회전.
        #   각속도는 잔여각 비례(0.18→0.06 하한)로 줄여 낡은 피드백(2Hz) 오버슛을 막는다.
        #   도달 후 hold 동안 검출 대기 — 재획득되면 중앙정렬로 전환, 없으면 그때 한바퀴(최후 폴백).
        self.declare_parameter('use_aimed_reacquire', True)
        self.declare_parameter('aimed_reacq_hold_sec', 1.5)  # 조준 도달 후 검출 대기[s] (2Hz면 3프레임)
        # ★SAFETY_EVENTS(7/13, 서버 가이드 연동): /robot{id}/server/safety_events 구독 —
        #   서버가 1인칭 시야 검출 중 "중앙 1/3과 60% 중첩 + 최소크기" 필터를 통과한 고신뢰
        #   이벤트만 발행(쿨다운 3s). 현재 임무 타입과 일치하면 center_px 로 bearing 을 계산해
        #   기존 bearing 채널에 주입 → 탐색 중지/중앙정렬/완료 판정은 기존 로직 그대로(새 분기 0).
        self.declare_parameter('use_server_safety_events', True)
        self.declare_parameter('safety_events_hfov_deg', 28.91)   # 캘리브 실측(fusion hfov_deg 와 짝)
        # ★SNAP_STOP(7/13): 고신뢰 보고 수신 = 촬영각 — 접근 즉시 중단하고 마무리
        self.declare_parameter('use_safety_event_stop', True)
        # ★STABLE_DONE(7/13, 사용자: "마지막까지 가운데 정렬한 후 멈춰야지"):
        #   중앙 판정이 이 시간 동안 (새 검출 포함) 유지돼야 완료 — 순간치에 굳지 않음.
        self.declare_parameter('center_stable_sec', 0.8)
        # ★AIM_OFFSET(7/13): 사진이 한쪽으로 일관되게 치우칠 때 보정 노브[deg, 좌+]
        self.declare_parameter('photo_aim_offset_deg', 2.5)   # ★7/13 실사진 튜닝(0→5→3.5→2.5, 아직 살짝 오른쪽 — 내일 마저)
        # ★AIM_OFFSET_PER_TYPE(7/14, 서버 팩트체크 회신 반영): center_px 역변환 무죄 확정 —
        #   잔여 우편향은 ①offset 자체가 수렴점을 22.1px/° 씩 민다(2.5°=+55px, FIRE 실측 일치)
        #   ②클래스별 bbox 비대칭(사람 어깨선/불꽃 기울기). → 타입별 오버라이드(999=공통값 사용).
        #   7/14 사진 4장 역산 추천값(🔴실기검증 전): fire -1.0 / fall -0.4 / no_helmet +1.6
        #   주행 중 라이브 튜닝: ros2 param set /robot1/patrol_commander photo_aim_offset_fire -1.0
        # ★7/14 박제(사용자 확정): 사진 4장 역산값을 기본값으로 — CLEAN_START 리셋에도 유지.
        self.declare_parameter('photo_aim_offset_fire', -1.0)
        self.declare_parameter('photo_aim_offset_fall', -0.4)
        self.declare_parameter('photo_aim_offset_no_helmet', 1.6)
        # ★GENTLE_TURN(7/13, 사용자: "팍팍 돌다 초점 나감 — 천천히 조금씩"):
        #   정렬 회전 전용 저속. 펄스=0.06rad/s(3.4°/s), 연속 상한=0.12rad/s.
        self.declare_parameter('fine_pulse_w', 0.06)
        self.declare_parameter('center_turn_max_w', 0.08)
        # ★PROXIMITY(7/13, 서버 구현완료 회신 연동): 글로벌캠 로봇간 근접경보.
        #   too_close(5Hz 레벨트리거) → 즉시 정지. 양보 규칙(우리 회신 그대로):
        #   이벤트 임무 중이 아닌 쪽이 물러남, 둘 다 순찰이면 robot_id 큰 쪽이 후진.
        #   cleared(8연발 재전송) 또는 too_close 끊김 proximity_timeout 초 → 하던 일 재개.
        self.declare_parameter('use_proximity_stop', True)
        self.declare_parameter('proximity_timeout', 300.0)  # ★7/15 사용자 확정: cleared 올 때까지 대기(순수 이벤트 구동). 300s=글로벌캠 사망시 최후 탈출구만. (서버가 5Hz 재발행 스펙 지키면 2.0 복귀 검토)
        self.declare_parameter('proximity_yield_v', -0.05)  # 양보측 후진 속도[m/s] (가드 통과 필수)
        # ★PROX_DETOUR(7/15, 서버문서 "즉시 제동 또는 우회 주행" 채택): cleared가 hold_max 내
        #   안 오면 대기 해제 → 기존 라이다 회피(감속0.40→정지→STUCK재시도→구간스킵)로 우회.
        #   재개 직후 too_close 재수신으로 도로 멈추는 핑퐁 방지 = snooze(그동안 라이다 가드가 안전).
        #   양보측(후진중)은 제외 — 후진하면 cleared가 자연히 온다.
        self.declare_parameter('use_proximity_detour', False)  # ★7/15 사용자 확정: 기본=정지→cleared→재개(순정). 우회는 봉인(필요시 True)
        self.declare_parameter('proximity_hold_max', 8.0)    # too_close 대기 상한[s] → 우회 전환
        self.declare_parameter('proximity_snooze', 20.0)     # 우회 전환 후 경보 무시 시간[s]
        # ★SCAN_WD(7/15 승인): scan 링크 워치독 — "눈 감고 달리기" 금지.
        #   주행 중 scan이 stale_sec 이상 끊기면 즉시 정지+PAUSED(SCAN_LOST), recover_sec 이상
        #   안정 복귀하면 하던 일 자동 재개. 오늘 벽박기·금지존 돌진·±35° 팔랑거림 전부
        #   "링크 사망 → 위치 썩은 채 주행"이 뿌리 — 링크가 죽으면 서 있는 게 유일한 안전.
        self.declare_parameter('use_scan_watchdog', True)
        self.declare_parameter('scan_stale_sec', 1.5)     # 이 시간 scan 무수신 = 실명 판정
        self.declare_parameter('scan_recover_sec', 3.0)   # 이 시간 연속 수신 = 회복 판정
        # ★SWAP+CAM_GATE: 랩도킹 완료=교대신호 / 카메라 게이트
        # ★7/17 사용자 확정: "카메라는 도킹존에서도 끄지 말고 항상 켜둔다" → 기본 False(=항상 ON 발행).
        #   True로 되돌리면 7/15 동작(대기 IDLE/CHARGING 시 OFF) 복귀.
        self.declare_parameter('use_lap_swap', True)
        self.declare_parameter('use_cam_gate', False)
        # ★DOCK_AIM(7/15 사용자 제안): DOCK 명령 전 마커(도크)방향 선조준 — SEARCH 맹회전 제거
        self.declare_parameter('use_dock_aim', False)  # ★7/15 저녁: 조준 무동작 미해결 — 원인 규명 전까지 봉인(도킹=어제 검증 그대로)
        # ★UNWEDGE(7/10, 사용자 요구 "막힌데 판단해서 안막힌곳 중 넓은 곳으로 조금씩 이동해서 빼기")
        self.declare_parameter('use_servo_unwedge', True)   # 끄면 옛 Nav2 ESCAPE만
        # ★NAV2_ESCAPE_OFF(7/10): Nav2 Spin/DriveOnHeading 폴백은 _rs_cmd를 안 거쳐
        #   마스크 하드가드가 적용되지 않는다(=금지존 무방비). 서보 탈출 타임아웃이 유도식으로
        #   충분해진 뒤로는 넘길 일이 없다. True 로 켜면 옛 폴백 부활(금지존 위험).
        #   ★7/13 기본 True 복귀(사용자 선택): raw goal 접근 중 서보 탈출이 못 빼는 교착 실측
        #     (하드가드 전진차단+이동 0.03m, 영구 정지 루프). Nav2 폴백으로 4/4회 탈출 성공,
        #     침범은 모서리 스침(≤22%, 발자국 1~2/9셀) 수준. 교착 방치가 더 나쁘다는 판단.
        self.declare_parameter('escape_nav2_fallback', True)
        self.declare_parameter('unwedge_clear_min', 0.28)   # 섹터 여유가 이 미만이면 막힘[m]
        self.declare_parameter('unwedge_look', 0.35)        # 금지존 검사 선분 길이[m]
        self.declare_parameter('unwedge_v', 0.05)           # 조금씩 이동 속도[m/s]
        self.declare_parameter('unwedge_w', 0.35)           # ★7/10: 0.20→0.35. 탈출은 카메라 블러 무관 — 느리면 타임아웃만 먹는다
        self.declare_parameter('unwedge_timeout', 15.0)     # 넘으면 Nav2 ESCAPE로 인계
        self.declare_parameter('unwedge_min_move', 0.15)    # 이만큼 '실제로' 이동해야 탈출 성공[m]
        # ★RTC_NEARDOCK(7/10): 충전소에서 이 반경 안이면 RTC가 Nav2를 건너뛰고 서보로 프리도킹 직행.
        #   빨강칸(충전기 인플레이션) 안에서 Nav2는 첫 틱부터 collision→abort 하므로 부르면 안 된다.
        self.declare_parameter('rtc_direct_radius', 0.8)
        self.declare_parameter('escape_goal_weight', 0.3)     # 목표방향 가중(클수록 목표쪽 선호)
        # ★WATCHDOG: goal 떠 있는데 실제로 안 움직이면(=Nav2가 abort도 안 내는 '조용한 멈춤') ESCAPE 강제
        self.declare_parameter('no_progress_timeout', 7.0)    # 이 초 동안 무이동이면 얼어붙음 판정
        self.declare_parameter('no_progress_min_dist', 0.05)  # 이 거리 이상 움직이면 '진전 있음'
        self.robot_id = self.get_parameter('robot_id').value
        self.max_retries = self.get_parameter('max_retries').value
        self.stuck_max_retries = self.get_parameter('stuck_max_retries').value
        self.stuck_retry_wait = self.get_parameter('stuck_retry_wait').value
        self.dispatch_max_retries = self.get_parameter('dispatch_max_retries').value
        # battery_threshold / charge_target_pct / charging_timeout 은 런타임 param set으로
        # 바꿔 테스트할 수 있게 사용 시점에 get_parameter로 즉시 읽음(캐시 안 함).

        # --- FSM/순찰 상태 ---
        self.current_state = PatrolState.IDLE
        self.pause_reason = ''
        self.waypoints = []
        self.charging_station = None        # ★BATTERY: yaml의 충전소 좌표
        self.current_waypoint_index = 0
        self.saved_waypoint_index = 0       # 이벤트/파견/수동/배터리 전 순찰 위치 저장
        self.current_battery = 100.0        # ★BATTERY: 아직 못 받았을 때 오판 방지용 초기값
        self.charging_start = None          # ★BATTERY Phase3: CHARGING 진입 시각(타임아웃용)
        self._batt_low_since = None         # ★BATT_IMMED: 순찰 중 저배터리 연속 시작 시각
        self._idle_dispatch_pending = False # ★IDLE_DISPATCH: 대기 중 파견 수락→언도크 후 임무 플래그
        self._rtc_docking = False           # ★감사픽스(7/7): 미초기화=PATROLLING 첫 틱 AttributeError 크래시(확정)
        self._handover_sent = False         # ★감사픽스: 교대요청 발행됨 → 충전완료 후 자동 재출격 금지(이중순찰 방지)
        self.charging_ref_battery = None    # ★CHARGE_FIX: 충전 상승 감시 기준 배터리%
        self.charging_ref_time = None       # ★CHARGE_FIX: 상승 감시 기준 시각(정체 판정용)
        # ★UNDOCK/DOCK: Pi 실행노드 명령/완료 추적
        self._dock_done_msg = None          # Pi가 보낸 최근 완료신호('UNDOCK_DONE'/'DOCK_DONE')
        self.undock_cmd_sent = False        # UNDOCKING에서 UNDOCK 명령 1회 발행 플래그
        self.undock_start = None            # UNDOCK 타임아웃 기준시각
        self.dock_cmd_sent = False          # DOCKING에서 DOCK 명령 1회 발행 플래그
        self.docking_start = None           # DOCK 타임아웃 기준시각
        self.retry_count = 0
        self.dispatch_target_wp = -1        # ★HANDOVER: 인계받아 이어갈 wp (그 외 -1)
        self.goal_in_flight = False
        self.current_goal_handle = None     # ★ESTOP_FIX 1: 진행 중 Nav2 goal 핸들 (취소용)
        self.dispatch_target = None         # 파견 목표 PoseStamped (있으면 MOVING_TO_EVENT)
        self.dispatch_retry_count = 0       # 파견 goal 실패 누적 (소진 시 충전소 복귀)
        self.obstacle_wait_start = None     # OBSTACLE_WAITING 진입 시각
        self.cm_stop_since = None           # ★OBSTACLE_BRIDGE: collision_monitor STOP 연속 시작 시각
        self.cm_clear_since = None          # ★OBSTACLE_BRIDGE: 전방 뚫림(비STOP) 연속 시작 시각
        self.stuck_retry_count = 0          # STUCK에서 몇 번 재시도했나
        self.stuck_wait_start = None        # STUCK 대기 시작 시각
        self.retry_wait_start = None        # RETRYING 대기 시작 시각
        # ★ESCAPE 상태
        self._costmap = None                # 최신 local costmap (OccupancyGrid)
        self._gcostmap = None               # ★최신 global costmap(map 프레임, keepout 포함)
        self._odom = None                   # (x, y, yaw) odom 프레임
        self._map_pose = None               # (x, y, yaw) map 프레임 (AMCL)
        self.escape_return_state = PatrolState.PATROLLING  # 탈출 성공 후 복귀 상태
        self.escape_attempts = 0
        self.escape_busy = False            # spin/drive 액션 진행 중
        self.escape_phase = None            # 'SPIN' / 'DRIVE'
        self._progress_ref = None           # ★WATCHDOG: 마지막 '진전' 위치(odom x,y)
        self._progress_since = None         # ★WATCHDOG: 진전 없이 머문 시작 시각
        self.dock_arrival_count = 0         # ★DWELL: 도킹존 도착 누적(2회째부터 대기)
        self.dock_dwell_start = None        # ★DWELL: 도킹 대기 시작 시각
        # ★ROUTE_SERVO 주행부 상태 (PATROLLING+게이트ON에서만 활동)
        self._rs_pts = None                 # 그래프 노드 {id:(x,y)} — 첫 사용 시 로드
        self._rs_adj = {}                   # ★GRAPH_ONLY(7/10): 엣지 인접표 {id:{id,...}} — 그래프 위로만 주행
        self._km = None                     # ★MASK_HARDGUARD(7/10): 금지존 마스크(코스트맵과 독립)
        self._rs_full = None                # 이번 랩 노드열(최근접 시작노드 포함). None=랩 미구성
        self._rs_leg = 0                    # 현재 구간 인덱스 (목표 = _rs_full[_rs_leg+1])
        self._rs_phase = None               # 'TURN' | 'DRIVE' | 'SETTLE'
        self._rs_phase_t0 = 0.0             # 페이즈 시작 시각(초)
        self._rs_settle = 0                 # SETTLE 남은 틱
        self._rs_next_phase = None          # SETTLE 뒤 갈 곳('DRIVE'|'ADVANCE')
        self._rs_drive_timeout = 0.0        # 현재 구간 DRIVE 제한시간
        self._rs_lv = 0.0                   # 가감속 램프용 직전 명령
        self._rs_lw = 0.0
        self._rs_front = None               # 전방(±25°) 라이다 최소거리
        self._rs_rear = None                # 후방(±25°) 라이다 최소거리 — BACKOFF 가드
        self._rs_seg_retries = 0            # 현재 구간 자체 재시도(2회 후 RETRYING 위임)
        self._rs_dock_retry = False         # ★True=도킹 실패 후 재시도 맥락 → 랩 생략하고 바로 도킹.
                                            #   (언도킹 직후엔 False — 종점 근처라도 정상 랩을 돎)
        self._rs_fresh = True               # ★True=다음 랩은 처음부터(전체 노드). False=중단 재개
                                            #   → 최근접 노드부터 잔여 구간만 이어달리기(헛바퀴 방지)
        self._rs_resume_node = None         # ★RESUME_SAVED(7/8): 이벤트로 순찰 이탈 시 '가던 노드'를 저장.
                                            #   재개 때 최근접(이벤트 위치)이 아니라 이 노드부터 이어감(발견 당시 위치부터).
        self._estop_prev = None             # ★ESTOP_RESUME(7/10): 비상정지 직전 상태 — RESUME으로 하던 일 이어받기.
                                            #   ESTOP은 _rs_stop()(0속도)만 부르고 _rs_reset()은 안 부르므로
                                            #   _rs_full/_rs_leg/dispatch_target이 살아있다 → 재개 재료는 이미 있다.
        self._rs_evt = False                # ★True=이벤트(파견) 접근 미션 수행중 (랩과 배타적)
                                            #   서보 순찰선=벽 근접이라 Nav2가 파견 goal을 즉시 거부
                                            #   (7/6 실주행 5연속) → 이벤트 접근도 서보 직진으로 수행
        # ★VISUAL_CENTER(7/8): 융합노드가 발행하는 활성이벤트 실시간 방위각(폐루프 중앙정렬용)
        self._event_bearing = None          # 최근 수신 방위각[rad, 좌+], 미수신=None
        self._bearing_buf = []              # ★BEARING_MED3(7/13): 최근 3샘플 중앙값(불 bbox 요동 흡수)
        self._event_bearing_t = 0.0         # 최근 수신 시각[s] (staleness 판정)
        self._center_bmin = None            # FACEYAW 중 관측된 |방위| 최소(발산=부호반대 감지용)
        self._event_centering = False       # ★NAV2_EVENT: Nav2 도착 후 폐루프 중앙정렬 진행중
        self._event_center_t0 = 0.0         # 중앙정렬 시작 시각[s]
        self._event_target_yaw = None       # ★CENTER_FALLBACK: 대상 방향 map-yaw(θ). bearing 유실 시 개루프 회전 폴백용
        # ★PREFACE(7/9): 파견 수락 직후 '즉시정지→정면 응시→접근' 선응시 단계
        self._event_preface = False         # True=선응시 진행중(완료 전 접근 goal 발사 보류)
        self._event_preface_t0 = 0.0        # 선응시 시작 시각[s]
        # ★EVENT_FINALIZE(7/9): 도착 마무리 파이프라인 상태
        self._event_final_phase = None      # None(비활성) | 'RANGE'(거리조정) | 'TRIM'(평행트림) | 'CENTER2'(최종 중앙정렬)
        self._reacq_yaw0 = None             # ★REACQUIRE(7/10): 재탐색 시작 yaw (None=재탐색 비활성)
        self._reacq_dir = 1.0               # 재탐색 훑는 방향(+1 → 끝나면 -1로 1왕복)
        self._reacq_t0 = 0.0                # 재탐색 시작 시각[s]
        self._reacq_cycle = 0               # ★SEARCH_CYCLES(7/10): 몇 번째 탐색 창인가
        self._det_last_t = -1e9             # ★DET_SLOW(7/17): 마지막 '비어있지 않은' 검출 수신 시각[s]
        self._chg_base = None               # ★CHARGE_OBS(7/17): 도크존 배터리 최저점(상승 감지 기준선)
        self._reacq_gaveup = False          # ★REACQ_LOOPFIX(7/10): 이 정렬 국면에서 이미 포기했나(재스윕 금지)
        self._reacq_accum = 0.0             # ★FULL_CIRCLE(7/13): 누적 회전각[rad] (한바퀴 판정)
        self._reacq_last_yaw = None         # ★FULL_CIRCLE(7/13): 직전 틱 yaw (wrap 안전 누적용)
        self._reacq_success_t = 0.0         # ★TIMER_RESET(7/13): 재획득 성공 시각(정렬시간 새로 재기)
        self._reacq_backup_p0 = None        # ★BACKUP_REACQ(7/13): 후진 시작 위치
        self._reacq_backup_done = False     # ★BACKUP_REACQ(7/13): 이번 유실에서 후진 시도 소진
        self._event_dir_map = None          # ★AIMED_REACQ(7/14): 마지막 목격 시점의 map 절대방향[rad]
        self._event_face_yaw = None         # ★PHOTO_SPOT(7/14): 파견 goal 의 응시방향(orientation yaw)
        self._reacq_aim_done = False        # ★AIMED_REACQ: 이번 유실에서 조준 시도 소진
        self._reacq_aim_hold_t0 = None      # ★AIMED_REACQ: 조준 도달 후 검출 대기 시작 시각
        self._reacq_aim_t0 = None           # ★AIMED_REACQ: 조준 시작 시각(안전 타임아웃용)
        self._event_force_nav2 = False      # ★HYBRID_FALLBACK(7/13): 이 이벤트만 Nav2 우회(서보 막힘 시)
        self._mg_block_first = None         # ★FRAME_SETTLE(7/13): 전진 가드 차단 시작 시각
        self._mg_block_last = None          # ★FRAME_SETTLE(7/13): 전진 가드 차단 최근 시각
        self._estop_release = False         # ★ESTOP_LATCH(7/13): 관제 해제창(RESUME/RESET만 True)
        self._snap_stop = False             # ★SNAP_STOP(7/13): 서버 신호 정지(거리조정 생략) 티켓
        self._center_hold_until = 0.0       # ★STOP_THEN_FIND(7/13): 정지 후 이 시각까지 완전 정지(찾기 전 안정)
        self._fc_spin_until = 0.0           # ★FINE_CENTER(7/13): 펄스 회전 종료 시각
        self._fc_wait_until = 0.0           # ★FINE_CENTER(7/13): 펄스 후 검출 대기 종료 시각
        self._fc_w = 0.0                    # ★FINE_CENTER(7/13): 현재 펄스 회전 속도
        self._cs_ok_since = 0.0             # ★STABLE_DONE(7/13): 중앙 상태 유지 시작 시각
        self._prox_active = False           # ★PROXIMITY(7/13): too_close 근접경보 활성 중
        self._prox_last_t = 0.0             # 마지막 too_close 수신 시각[s]
        self._prox_start_t = 0.0            # ★PROX_DETOUR(7/15): 이번 경보 시작 시각(hold_max 판정)
        self._prox_snooze_until = 0.0       # ★PROX_DETOUR(7/15): 이 시각까지 too_close 무시(우회중)
        self._dock_aim_start = None         # ★DOCK_AIM(7/15): 조준 시작 시각(타임아웃 판정)
        self._scan_last_arrival = None      # ★SCAN_WD(7/15): 마지막 scan 수신 시각[s]
        self._scanwd_active = False         # ★SCAN_WD: 실명 정지 중
        self._scanwd_prev = None            # 실명 직전 상태(재개용)
        self._scanwd_ok_since = None        # scan 연속수신 시작 시각(회복 판정)
        self._prox_prev = None              # 경보 직전 상태(재개용)
        self._prox_yield = False            # 이번 경보에서 우리가 물러나는 쪽인가
        self._event_target_raw = None       # ★EARLY_STOP(7/10): 대상 '원위치'(standoff 적용 전) map 좌표
        self._rs_sect = None                # ★UNWEDGE(7/10): 12섹터 라이다 최소거리
        self._uw_t0 = 0.0                   # ★UNWEDGE: 서보 탈출 시작 시각(0=비활성)
        self._uw_p0 = None                  # ★UNWEDGE: 탈출 시작 위치 — 실제 이동량 판정용
        self._uw_gaveup = False             # ★UNWEDGE: 서보 포기 → 이 국면은 Nav2 ESCAPE가 담당
        self._event_yaw_pre_trim = None     # ★TRIM_UNDO: 평행트림 직전 yaw(대상 유실 시 복귀용)
        self._event_phase_t0 = 0.0          # 현 단계 시작 시각[s]
        self._rs_front_ctr = None           # 전방 협각(±8°) 중앙값 거리 — _rs_scan_cb가 갱신
        self._last_scan = None              # 최신 LaserScan(벽피팅용) — _rs_scan_cb가 갱신
        # ★CLEAR_DETOUR 상태 (use_clear_detour 게이트 ON일 때만 값이 바뀜)
        self._cd_block_since = None         # 서보 DRIVE 전방막힘 연속 시작 시각(초, _rs_now_sec)
        self._cd_target_xy = None           # 막힌 구간의 도착노드 좌표 (CLEAR 시 Nav2 우회 목표)
        self._cd_target_node = None         # 위 노드 id (로그용)
        self._cd_fallback_node = None       # ★DYN_OBS: 우회 거부 시 폴백(다음다음 노드) id
        self._cd_fallback_xy = None         # ★DYN_OBS: 위 노드 좌표
        self._cd_detour_active = False      # True=CLEAR 우회 Nav2 goal 진행중 (PATROLLING에서만 유효)
        self._cd_backoff_t0 = None          # ★DETOUR_BACKOFF(7/22): 우회 goal 발사 전 후진 간격확보 시작시각(float초). None=후진 아님
        self._cd_clear_since = None         # OBSTACLE_WAITING 전방 뚫림(서보 라이다) 연속 시작 시각
        self._cd_auto_detour_used = False   # True=이번 막힘에서 verdict 무응답 자율우회 이미 1회 사용
                                            #   (같은 막힘 재무응답 시엔 STUCK — 무한루프 방지.
                                            #    진전(구간 완료/우회 성공/뚫림/CLEAR 수신) 시 리셋)

        self.load_waypoints()

        # --- Nav2 액션 클라이언트 (상대경로: namespace로 분리) ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # ★ESCAPE: 복구 이동용 (behavior_server=루트 네임스페이스라 절대경로)
        self._spin_cli = ActionClient(self, Spin, '/spin')
        self._drive_cli = ActionClient(self, DriveOnHeading, '/drive_on_heading')

        # --- /state 발행 (latched: status_reporter가 늦게 떠도 최신 상태 확보) ---
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, 'state', state_qos)
        # ★NAV_REPORT(7/14, 관제 GUI 약속분): FSM·랩 노드열·ESCAPE·전방거리 등을 JSON 으로
        #   status_reporter 에 넘기는 내부 토픽. status_reporter 가 AMCL/lifecycle 과 병합해
        #   /robotN/nav_report 로 최종 발행한다. 여기선 '아는 것만' 낸다 — FSM 무손상(발행 전용).
        self.nav_progress_pub = self.create_publisher(String, 'nav_progress', state_qos)
        self._last_goal_status = ''
        self.create_timer(0.5, self.publish_nav_progress)   # 2Hz(장애물 거리 등 연속값 갱신)
        # ★MISSION_TYPE(7/13): 현재 임무 타입 방송(latched) — fusion 이 이 클래스만 추적.
        #   (NO_HELMET 임무 중 fusion 이 FALL 을 물어 goal 오염·눈뜬장님 탐색 실증 → 타입 핸드셰이크)
        self.active_type_pub = self.create_publisher(String, 'event/active_type', state_qos)

        # ★DRIFT_RESET: 출발/매 루프복귀 시 /initialpose(0,0,0) 재주입 → AMCL 누적 드리프트 리셋
        #   (RViz "Store initial_pose"와 동일 효과. 절대토픽 '/initialpose' = AMCL 루트 구독)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ★HANDOVER: 배터리 부족 시 2호기 교대요청 발행(값=robot2가 재개할 wp). 서버가 구독→robot2 지시.
        self.handover_pub = self.create_publisher(Int32, 'handover_request', 10)
        # ★CAM_GATE(7/15, 사용자 확정: "카메라는 패트롤 돌 때만"): 도크 대기(IDLE/CHARGING)면
        #   카메라 OFF, 그 외 전부 ON. Pi의 camera_gate.py가 구독해 센더를 켜고 끔.
        #   transient_local=Pi 게이트가 늦게 붙어도 마지막 상태 수신. 2s 주기 재발행(유실 방어).
        _cam_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=QoSReliabilityPolicy.RELIABLE)
        self.cam_enable_pub = self.create_publisher(Bool, 'cam_enable', _cam_qos)
        self.create_timer(2.0, self._cam_gate_tick)
        # ★UNDOCK/DOCK: Pi 로컬 실행노드로 명령 발행 / 완료신호 구독 (모션·정지는 Pi 로컬)
        self.dock_cmd_pub = self.create_publisher(String, 'dock_cmd', 10)   # 'UNDOCK' | 'DOCK'
        self.create_subscription(String, 'dock_done', self._on_dock_done, 10)  # 'UNDOCK_DONE' | 'DOCK_DONE'

        # ★DOCK_TOL: controller_server의 goal_checker.xy_goal_tolerance를 wp별로 동적 변경
        self._tol_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self._last_tol = None   # 중복 set 방지용

        # --- 입력: 사람 명령(서비스) / AI 판정(토픽) / 파견(서비스) / 배터리(토픽) ---
        self.set_mode_srv = self.create_service(SetMode, 'set_mode', self.handle_set_mode)
        self.dispatch_srv = self.create_service(
            DispatchToEvent, 'dispatch_to_event', self.handle_dispatch)
        self.create_subscription(
            ObstacleVerdict, 'obstacle_event', self.handle_obstacle_event, 10)
        self.create_subscription(
            BatteryState, 'battery_state', self.update_battery, 10)   # ★BATTERY
        # ★OBSTACLE_BRIDGE: 라이다 정지 신호 (Nav2 collision_monitor, 글로벌 토픽 → launch에서 remap)
        # 2026-06-28 임시 비활성화 — 순수 순찰 검증용. cm_stop_since가 늘 None이라 기존 순찰과 동일.
        #   순찰 안정화 후 장애물 로직 다시 설계하며 이 구독 복구.
        # self.create_subscription(
        #     CollisionMonitorState, 'collision_monitor_state',
        #     self.handle_collision_state, 10)

        # ★ESCAPE: local costmap / odom / amcl 구독 (절대경로 — 루트 토픽)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_costmap, 1)
        # ★KEEPOUT_SERVO(7/9): 노드 좌표는 map 프레임 → 진입 레그 검사엔 전역 코스트맵이 필요.
        #   (local_costmap 은 odom 프레임이라 map 좌표와 섞을 수 없음)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_gcostmap, 1)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        # ★ROUTE_SERVO 리소스: TF 피드백 + 전방가드 + 직접 주행명령 + 20Hz 틱.
        #   게이트 OFF면 틱이 즉시 리턴(비용 무시 수준) — 리소스는 상시 생성해 런타임 토글 지원.
        self._rs_cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._rs_scan_cb, qos_profile_sensor_data)
        # ★VISUAL_CENTER(7/8): 융합노드 실시간 방위각 구독 → 도착 후 폐루프 중앙정렬
        self.create_subscription(Float32, 'event/bearing', self._on_event_bearing, 10)
        # ★SAFETY_EVENTS(7/13): 네임스페이스(/robot1) 안이라 상대 이름 → /robot1/server/safety_events
        self.create_subscription(String, 'server/safety_events', self._on_safety_event, 10)
        # ★DET_SLOW(7/17): 서버 원본 검출 스트림(fusion 과 같은 토픽) — 감속 판단 전용.
        #   파견 판단은 그대로 fusion 몫. 여기선 '지금 뭔가 보인다'만 읽는다.
        self.create_subscription(String, 'safety/detections', self._on_detections_slow, 10)
        # ★PROXIMITY(7/13): 글로벌캠 근접경보(bridge_88 경유, 절대경로 — 네임스페이스 밖)
        self.create_subscription(
            String, '/globalcam/turtlebot_proximity/alerts', self._on_proximity, 10)
        self.create_timer(0.5, self._prox_timeout_tick)   # too_close 스트림 끊김 감시
        self.create_timer(0.5, self._scan_watchdog_tick)  # ★SCAN_WD(7/15): scan 링크 실명 감시
        self.create_timer(2.0, self._charge_watch_tick)   # ★CHARGE_OBS(7/17): 도크존 배터리 상승 = CHARGING
        self._rs_tfb = tf2_ros.Buffer()
        self._rs_tfl = tf2_ros.TransformListener(self._rs_tfb, self)
        self._load_keepout_mask()   # ★MASK_HARDGUARD: 주행 타이머 돌기 전에 반드시 적재
        self.create_timer(1.0 / RS_RATE, self._rs_tick)
        # ★ESTOP_FIX(7/10): /cmd_vel에 중재자(twist mux)가 없어 patrol·Nav2(collision_monitor)가
        #   같은 토픽을 두고 경쟁한다. 20Hz 0속도는 20Hz 잔여속도와 반반 섞여 로봇이 절뚝이며 굴러갔다.
        #   비상정지 동안만 100Hz로 0을 박아 경쟁에서 확실히 이긴다(그 외 상태에선 아무것도 안 함).
        self.create_timer(0.01, self._estop_hold_tick)

        self.create_timer(0.5, self.run_state_loop)   # 2Hz
        self.publish_state()                           # 초기 IDLE 1회 발행
        self.get_logger().info(
            f'PatrolCommander up | robot_id={self.robot_id} (set_mode PATROL_START 대기)')

    # ================================================================
    # 로딩 / 헬퍼
    # ================================================================
    def load_waypoints(self):
        pkg = get_package_share_directory('teamproject_navigation')
        yaml_path = os.path.join(pkg, 'config', 'waypoints.yaml')
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            self.waypoints = data['patrol_waypoints']
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            self.charging_station = data.get('charging_station')   # ★BATTERY
            if self.charging_station is not None:
                self.get_logger().info(
                    f"Charging station: "
                    f"({self.charging_station['x']}, {self.charging_station['y']})")
            else:
                self.get_logger().warn('charging_station 좌표 없음 (waypoints.yaml 확인)')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    def build_pose_from_waypoint(self, wp):
        # wp = dict(x, y, yaw_z, yaw_w). 순찰 wp / 충전소 둘 다 같은 키라 재사용 가능.
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=float(wp['x']), y=float(wp['y']), z=0.0)
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=float(wp['yaw_z']), w=float(wp['yaw_w']))
        return pose

    def publish_initialpose_if_docked(self, where=''):
        """★픽스B(7/15 승인): 도킹 확정 시 위치 주입도 라이다 도크 시그니처 검사 후에만.
        가짜 도킹(로봇1 실사고: 0cm 후진 DOCK_DONE)이 와도 엉뚱한 주입으로 AMCL을 죽이지 않는다.
        scan 미수신이면 옛 동작(주입) — 링크 죽음은 별개 문제."""
        rear = getattr(self, '_rs_rear', None)
        sig_max = float(self.get_parameter('dock_sig_rear_max').value)
        if rear is not None and rear > sig_max:
            self.get_logger().error(
                f'★DOCK_SIG 불일치({where}): 뒷벽 {rear:.2f}m > {sig_max:.2f}m — 실물이 도크에 없음! '
                f'위치 주입 생략(AMCL 보호). 도킹 실패 의심 — 실물 확인 필요')
            return False
        self.publish_initialpose()
        return True

    def publish_initialpose(self):
        """★DRIFT_RESET: /initialpose 에 (0,0,0) 재주입해 AMCL 누적 드리프트 리셋.
        전제: 호출 시점에 로봇이 실제로 시작테이프(0,0, yaw0)에 있어야 유효
        (PATROL_START 직후 / 마지막 wp=원점 도착 직후)."""
        # ★DOCK_POSE(7/14): 로봇별 도크 좌표 사용 (로봇1 기본 0,0,0 — 동작 불변)
        import math as _m
        _dx = float(self.get_parameter('dock_x').value)
        _dy = float(self.get_parameter('dock_y').value)
        _dyaw = float(self.get_parameter('dock_yaw').value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position = Point(x=_dx, y=_dy, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=_m.sin(_dyaw / 2), w=_m.cos(_dyaw / 2))
        cov = [0.0] * 36
        cov[0] = 0.0625    # x 분산 (0.25^2)
        cov[7] = 0.0625    # y 분산
        cov[35] = 0.0685   # yaw 분산
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'↻ /initialpose ({_dx:.2f},{_dy:.2f},{_dyaw:.2f}) 재주입 — 드리프트 리셋')

    def set_goal_tolerance(self, xy, yaw=None):
        """★DOCK_TOL/DOCK_YAW: controller_server goal_checker의 xy·yaw 도착오차를 동적 set.
        같은 값 반복 set 방지. 서비스 미준비/실패해도 그냥 넘어감(파일 기본값 유지)."""
        xy = round(float(xy), 4)
        yaw = round(float(yaw), 4) if yaw is not None else None
        key = (xy, yaw)
        if key == self._last_tol:
            return
        if not self._tol_cli.service_is_ready():
            return
        params = []
        pxy = ParamMsg()
        pxy.name = 'goal_checker.xy_goal_tolerance'
        pxy.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xy)
        params.append(pxy)
        if yaw is not None:
            pyaw = ParamMsg()
            pyaw.name = 'goal_checker.yaw_goal_tolerance'
            pyaw.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw)
            params.append(pyaw)
        req = SetParameters.Request()
        req.parameters = params
        self._tol_cli.call_async(req)
        self._last_tol = key
        self.get_logger().info(
            f'도착오차 → xy={xy}m' + (f', yaw={yaw}rad' if yaw is not None else ''))

    def update_battery(self, msg):
        """★BATTERY: 펌웨어 percentage 그대로 사용 (status_reporter와 동일 처리).
        0~1 스케일이면 *100, 이미 0~100이면 그대로. NaN은 무시(직전 값 유지)."""
        pct = msg.percentage
        if pct is not None and pct == pct:   # pct == pct: NaN이면 False → NaN 거르기
            self.current_battery = pct * 100.0 if pct <= 1.0 else pct

    def change_state(self, new_state, reason=''):
        """상태 전이 + 즉시 /state 발행 (전이 순간을 status_reporter가 바로 받게)."""
        # ★★ESTOP_LATCH(7/13, 1순위 절대규칙): 비상정지는 관제(RESUME/RESET)만 푼다.
        #   실측: ESTOP 9초 뒤 탈출 루프가 '성공 → RTC 복귀'로 래치를 덮어쓰고 스스로 주행 재개.
        #   개별 누수 지점을 쫓는 대신 상태 전이의 유일한 관문인 여기서 전부 차단
        #   (_rs_cmd 의 이동 게이트와 같은 초크포인트 원칙).
        if (self.current_state == PatrolState.EMERGENCY_STOP
                and new_state != PatrolState.EMERGENCY_STOP
                and not getattr(self, '_estop_release', False)):
            self.get_logger().error(
                f'★ESTOP 래치: {new_state.name} 전이 거부 — 관제 RESUME/RESET 만 해제 가능',
                throttle_duration_sec=2.0)
            return
        # ★NAV2_EVENT(7/8): MOVING_TO_EVENT 벗어나면 중앙정렬 플래그 해제(RESET/RESUME/실패/PAUSED 안전)
        if new_state != PatrolState.MOVING_TO_EVENT:
            self._event_centering = False
            self._event_final_phase = None   # ★EVENT_FINALIZE: 마무리 단계도 함께 해제
            self._event_yaw_pre_trim = None  # ★TRIM_UNDO 리셋
            self._event_preface = False      # ★PREFACE: 선응시도 함께 해제(RESET/ESTOP 안전)
        elif self.current_state != PatrolState.MOVING_TO_EVENT:
            # ★PREFACE(7/9): MOVING_TO_EVENT 신규 진입(파견수락/언도크후/ESCAPE복귀) 공통 —
            #   접근 goal 발사 전에 '정지→이벤트 정면 응시'부터. _rs_tick이 수행.
            self._event_preface = True
            self._event_preface_t0 = self._rs_now_sec()
            self._center_bmin = None
            self._event_dir_map = None   # ★AIMED_REACQ(7/14): 이전 이벤트 목격방향 잔재 제거
        self.current_state = new_state
        self.pause_reason = reason if new_state == PatrolState.PAUSED else ''
        self.publish_state()

    def compute_target_wp(self):
        """현재 상태 기준으로 status_reporter에 알릴 목표 wp 인덱스.
        순찰 미션 중 → 향하는 wp / 이벤트·충전으로 벗어남 → 재개·인계할 wp / 그 외 → -1"""
        s = self.current_state
        PATROL = (PatrolState.PATROLLING, PatrolState.ARRIVED, PatrolState.RETRYING,
                  PatrolState.STUCK, PatrolState.OBSTACLE_WAITING, PatrolState.RESUMING,
                  PatrolState.ESCAPE, PatrolState.DOCK_DWELL, PatrolState.DOCKING,
                  PatrolState.UNDOCKING)
        SAVED = (PatrolState.PAUSED, PatrolState.MOVING_TO_EVENT, PatrolState.LOW_BATTERY,
                 PatrolState.RETURNING_TO_CHARGER, PatrolState.CHARGING,
                 PatrolState.RESUMING_AFTER_CHARGE)
        if s in PATROL:
            # ★WP_REPORT(7/13, 사용자: "서버에 1하고 22밖에 안 떠"): 서보 순찰은
            #   current_waypoint_index 를 안 써서 1에 고정돼 있었다 — 실제 목표(다음 route 노드) 보고.
            if self._rs_full is not None and self._rs_leg + 1 < len(self._rs_full):
                return int(self._rs_full[self._rs_leg + 1])
            return self.current_waypoint_index
        if s in SAVED:
            # 이벤트/복귀 중엔 '재개 예정 노드'가 실제 의미 있는 목표
            if self._rs_resume_node is not None:
                return int(self._rs_resume_node)
            return self.saved_waypoint_index
        return -1   # IDLE, LOCALIZING, MANUAL_CONTROL, EMERGENCY_STOP

    def publish_state(self):
        msg = String()
        reason = self.pause_reason if (self.current_state == PatrolState.PAUSED
                                       and self.pause_reason) else ''
        target_wp = self.compute_target_wp()
        # 형식: "STATE|reason|index" 3필드 고정 (reason은 PAUSED일 때만 채워짐)
        msg.data = f'{self.current_state.name}|{reason}|{target_wp}'
        self.state_pub.publish(msg)
        self.publish_nav_progress()   # ★NAV_REPORT: 상태 전이 시 즉시 반영(2Hz 타이머와 별개)

    def publish_nav_progress(self):
        """★NAV_REPORT(7/14): 관제 GUI용 내부 스냅샷(JSON). patrol_commander 가 아는 것만 —
        route/FSM/recovery/장애물. AMCL·lifecycle 은 status_reporter 가 붙인다.
        방어: 초기화 순서·미생성 속성과 무관하게 절대 FSM 을 죽이지 않는다(전체 try)."""
        try:
            lap = list(self._rs_full) if getattr(self, '_rs_full', None) else None
            leg = int(getattr(self, '_rs_leg', 0))
            pts = getattr(self, '_rs_pts', None) or {}
            # 랩 노드 좌표(관제가 지도에 그릴 수 있게). 노드 수 ~22개 = 부담 없음.
            nodes_xy = {str(k): [round(v[0], 3), round(v[1], 3)] for k, v in pts.items()}
            snap = {
                'fsm': self.current_state.name,
                'pause_reason': self.pause_reason or '',
                'lap': lap,                                   # 이번 랩 노드열(None=미구성)
                'leg': leg,                                   # 현재 구간(목표=lap[leg+1])
                'target_node': self.compute_target_wp(),
                'resume_node': getattr(self, '_rs_resume_node', None),
                'nodes_xy': nodes_xy,
                'goal_result': self._last_goal_status,
                'replan_count': int(self.retry_count) + int(getattr(self, '_rs_seg_retries', 0)),
                'escape_phase': self.escape_phase,            # 'SPIN'/'DRIVE'/None
                'escape_attempts': int(self.escape_attempts),
                'detour_active': bool(getattr(self, '_cd_detour_active', False)),
                'front_dist': (round(self._rs_front, 3)
                               if getattr(self, '_rs_front', None) is not None else None),
            }
            m = String()
            m.data = json.dumps(snap, ensure_ascii=False)
            self.nav_progress_pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f'nav_progress 발행 실패(무해): {e}',
                                   throttle_duration_sec=10.0)

    # ================================================================
    # Nav2 goal 송수신
    # ================================================================
    def send_nav_goal(self, pose):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready, waiting...')
            return  # 다음 루프에서 재시도 (블로킹 금지)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_in_flight = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.handle_goal_response)

    def cancel_current_goal(self):
        """★ESTOP_FIX 2A: 진행 중인 Nav2 goal을 취소 (EMERGENCY_STOP/MANUAL 진입 시).
        goal_in_flight도 같이 풀어줘서 RESUME 후 다음 goal이 정상 발사되게 한다."""
        if self.current_goal_handle is not None:
            self.get_logger().info('진행 중 goal 취소 요청')
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.goal_in_flight = False

    # ================================================================
    # ★ESCAPE — 빨강(인플레이션/금지존) 탈출 복구
    #   goal 반복 실패(벽/금지존에 물림) → 뚫린 쪽으로 조금 빼서 빨강 벗어난 뒤
    #   원래 가려던 목표로 재계획. "무조건 후진" 아님(금지존은 costmap 高cost라 자동 배제).
    # ================================================================
    def _on_costmap(self, msg):
        self._costmap = msg

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self._odom = (p.x, p.y, yaw)

    def _on_amcl(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self._map_pose = (p.x, p.y, yaw)

    @staticmethod
    def _norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _robot_cell_cost(self):
        """로봇 현재 위치의 local costmap cost(0~100). 못 읽으면 None."""
        cm = self._costmap
        if cm is None or self._odom is None:
            return None
        info = cm.info
        cx = int((self._odom[0] - info.origin.position.x) / info.resolution)
        cy = int((self._odom[1] - info.origin.position.y) / info.resolution)
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return cm.data[cy * info.width + cx]

    def _in_red(self):
        """로봇이 빨강(막힘) 안에 있나? costmap 못 읽으면 False(탈출한 셈 치고 재계획)."""
        v = self._robot_cell_cost()
        if v is None:
            return False
        return v >= self.get_parameter('escape_cost_block').value

    def _goal_relative_bearing(self):
        """복귀 목표(순찰 wp 또는 충전소) 방향을 로봇 정면 기준 상대각으로. 없으면 None."""
        if self._map_pose is None:
            return None
        if self.escape_return_state == PatrolState.RETURNING_TO_CHARGER:
            tgt = self.charging_station
        elif 0 <= self.current_waypoint_index < len(self.waypoints):
            tgt = self.waypoints[self.current_waypoint_index]
        else:
            tgt = None
        if tgt is None:
            return None
        mx, my, myaw = self._map_pose
        bearing = math.atan2(float(tgt['y']) - my, float(tgt['x']) - mx)
        return self._norm(bearing - myaw)

    def compute_escape_heading(self):
        """local costmap에서 가장 뚫린(+목표방향 가중) 방향을 로봇 정면 기준 상대각으로 반환.
        사방이 막혔으면 None. 금지존은 costmap 高cost라 자동 배제됨."""
        cm = self._costmap
        if cm is None or self._odom is None:
            return None
        info = cm.info
        res = info.resolution
        W, H = info.width, info.height
        data = cm.data
        rc = int((self._odom[0] - info.origin.position.x) / res)
        rr = int((self._odom[1] - info.origin.position.y) / res)
        block = self.get_parameter('escape_cost_block').value
        probe = max(1, int(self.get_parameter('escape_probe_radius').value / res))
        ndirs = self.get_parameter('escape_num_dirs').value
        w = self.get_parameter('escape_goal_weight').value
        need = self.get_parameter('escape_distance').value
        goal_rel = self._goal_relative_bearing()
        ryaw = self._odom[2]
        best = None
        for i in range(ndirs):
            theta = 2.0 * math.pi * i / ndirs      # costmap(odom) 절대각
            ct, st = math.cos(theta), math.sin(theta)
            clear = 0
            for step in range(1, probe + 1):
                cx = int(rc + step * ct)
                cy = int(rr + step * st)
                if cx < 0 or cy < 0 or cx >= W or cy >= H:
                    break
                v = data[cy * W + cx]
                if v < 0 or v >= block:            # 미탐색/막힘 → 여기서 끊김
                    break
                clear = step
            clear_m = clear * res
            rel = self._norm(theta - ryaw)          # 로봇 정면 기준 상대각
            score = clear_m
            if goal_rel is not None:
                score -= w * abs(self._norm(rel - goal_rel))
            if best is None or score > best[0]:
                best = (score, rel, clear_m)
        if best is None or best[2] < need:          # 최소 전진거리만큼도 안 뚫림
            return None
        return best[1]

    def _enter_escape(self, return_state):
        """빨강 탈출 시퀀스 시작 (진행 중 goal 취소 후 ESCAPE 진입)."""
        self.cancel_current_goal()
        self._uw_t0 = 0.0            # ★UNWEDGE_FIX: 새 ESCAPE 국면 — 서보 탈출 상태 초기화
        self._uw_p0 = None
        self._uw_gaveup = False      # (초기화 안 하면 한 번 포기 후 영영 서보 탈출이 안 돈다)
        self.escape_return_state = return_state
        self.escape_attempts = 0
        self.escape_busy = False
        self.escape_phase = None
        self.change_state(PatrolState.ESCAPE)

    def _reset_escape(self):
        self.escape_attempts = 0
        self.escape_busy = False
        self.escape_phase = None

    def _finish_escape_resume(self):
        rs = self.escape_return_state
        self.get_logger().info(f'ESCAPE 성공 → {rs.name} 복귀(재계획)')
        self._reset_escape()
        self.change_state(rs)

    def _run_escape(self):
        """ESCAPE 상태 루프(2Hz). spin/drive 액션 진행 중이면 대기."""
        # ★UNWEDGE_FIX(7/10): 서보 탈출이 켜져 있으면 Nav2 Spin/DriveOnHeading을 발사하지 않는다.
        #   둘 다 돌면 /cmd_vel 을 두고 싸우며 서로 다른 방향으로 돌린다(실주행 실증:
        #   'ESCAPE 시도 -140°' + '★탈출 방향 +60°' 동시 출력). 서보가 타임아웃으로 포기하면
        #   _uw_gaveup 을 세워 여기서 옛 경로를 재개한다.
        if self.get_parameter('use_servo_unwedge').value and not self._uw_gaveup:
            return
        if (self._uw_gaveup
                and not self.get_parameter('escape_nav2_fallback').value):
            # 서보가 포기했지만 Nav2 폴백은 금지존 무방비 → 정지 유지(관제 확인).
            self._rs_stop()
            self.get_logger().error(
                '★서보 탈출 실패 + Nav2 폴백 비활성 → 정지 유지(금지존 무방비 회피). 관제 확인 필요',
                throttle_duration_sec=3.0)
            return
        if self.escape_busy:
            return
        # 최소 1회 탈출 이동을 한 뒤에만 '빠져나옴' 판정. (무이동 워치독으로 온 경우 red가
        #  아니어도 일단 한 번은 움직여서 컨트롤러 교착을 풀어줘야 함.)
        if self.escape_attempts > 0 and not self._in_red():
            self._finish_escape_resume()
            return
        if self.escape_attempts >= self.get_parameter('escape_max_attempts').value:
            self.get_logger().error('ESCAPE 소진 — 탈출 실패 → STUCK')
            self._reset_escape()
            self.stuck_retry_count = 0
            self.change_state(PatrolState.STUCK)
            return
        heading = self.compute_escape_heading()
        if heading is None:
            self.get_logger().error('ESCAPE — 뚫린 방향 없음(사방 막힘) → STUCK')
            self._reset_escape()
            self.stuck_retry_count = 0
            self.change_state(PatrolState.STUCK)
            return
        self.escape_attempts += 1
        self.get_logger().warn(
            f'ESCAPE 시도 {self.escape_attempts} — 상대 {math.degrees(heading):.0f}°로 '
            f'회전 후 {self.get_parameter("escape_distance").value}m 전진')
        self._send_spin(heading)

    def _send_spin(self, heading):
        if not self._spin_cli.server_is_ready():
            self.get_logger().warn('ESCAPE: /spin 서버 미준비 — 다음 루프 재시도')
            return
        self.escape_busy = True
        self.escape_phase = 'SPIN'
        g = Spin.Goal()
        g.target_yaw = float(heading)
        g.time_allowance = MsgDuration(sec=15)
        self._spin_cli.send_goal_async(g).add_done_callback(self._spin_goal_resp)

    def _spin_goal_resp(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('ESCAPE: spin 거부 — 다음 루프 재시도')
            self.escape_busy = False
            return
        gh.get_result_async().add_done_callback(self._spin_done)

    def _spin_done(self, future):
        # ★감사픽스(7/7): RESET/ESTOP 등으로 ESCAPE를 이탈했으면 전진 체인 중단
        #   (구버전: IDLE/EMERGENCY_STOP 상태에서도 DriveOnHeading 0.25m 발사됨)
        if self.current_state != PatrolState.ESCAPE:
            self.get_logger().info('ESCAPE: 회전 결과 무시(상태 이탈) — 전진 체인 중단')
            self.escape_busy = False
            return
        self.get_logger().info('ESCAPE: 회전 완료 → 전진')
        self._send_drive()

    def _send_drive(self):
        if not self._drive_cli.server_is_ready():
            self.get_logger().warn('ESCAPE: /drive_on_heading 서버 미준비 — 다음 루프 재시도')
            self.escape_busy = False
            return
        self.escape_phase = 'DRIVE'
        g = DriveOnHeading.Goal()
        g.target = Point(x=float(self.get_parameter('escape_distance').value), y=0.0, z=0.0)
        g.speed = float(self.get_parameter('escape_speed').value)
        g.time_allowance = MsgDuration(sec=15)
        self._drive_cli.send_goal_async(g).add_done_callback(self._drive_goal_resp)

    def _drive_goal_resp(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('ESCAPE: drive 거부 — 다음 루프 재시도')
            self.escape_busy = False
            return
        gh.get_result_async().add_done_callback(self._drive_done)

    def _drive_done(self, future):
        self.get_logger().info('ESCAPE: 전진 완료 → 재평가')
        self.escape_busy = False
        self.escape_phase = None

    def _update_stuck_watchdog(self):
        """★WATCHDOG: goal 실행 중인데 실제로 안 움직이면(Nav2 abort도 안 나는 '조용한 멈춤')
        일정 시간 후 ESCAPE 강제 발동. odom 위치로 실제 이동 여부 판단(AMCL은 떨려서 부적합)."""
        MOVING = (PatrolState.PATROLLING, PatrolState.RETURNING_TO_CHARGER,
                  PatrolState.MOVING_TO_EVENT)
        if (self.current_state not in MOVING or not self.goal_in_flight
                or self._odom is None):
            self._progress_ref = None
            self._progress_since = None
            return
        now = self.get_clock().now()
        x, y = self._odom[0], self._odom[1]
        if self._progress_ref is None:
            self._progress_ref = (x, y)
            self._progress_since = now
            return
        moved = math.hypot(x - self._progress_ref[0], y - self._progress_ref[1])
        if moved >= self.get_parameter('no_progress_min_dist').value:
            self._progress_ref = (x, y)          # 진전 있음 → 기준·타이머 갱신
            self._progress_since = now
            return
        stalled = (now - self._progress_since).nanoseconds / 1e9
        _to = self.get_parameter('no_progress_timeout').value
        # ★DETOUR_GRACE(7/22): 우회 goal 은 장애물 코앞 출발이라 RPP 초기 경로계산+제자리
        #   회전(변위 0)이 길다 — 7s 워치독이 우회를 끊고 ESCAPE 로 보내는 것 실측. 12s 유예.
        if self._cd_detour_active:
            _to = max(_to, 12.0)
        if stalled >= _to:
            self.get_logger().warn(
                f'무이동 {stalled:.1f}s (goal 실행중인데 안 움직임) → ESCAPE 강제 발동')
            rs = self.current_state
            self._progress_ref = None
            self._progress_since = None
            self._enter_escape(rs)

    def handle_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_in_flight = False
            # ★CLEAR_DETOUR: 우회 goal 거부(서보 순찰선=벽 0.2m라 인플레이션권 거부 가능)
            if self._cd_detour_active:
                # ★DYN_OBS(7/7): 포기 전에 '다음다음 노드'로 1회 폴백 재시도.
                #   도착노드가 벽에 붙어 거부돼도 한 구간 뒤 노드는 통과 가능할 수 있음.
                if self._cd_fallback_xy is not None:
                    self._cd_target_node = self._cd_fallback_node
                    self._cd_target_xy = self._cd_fallback_xy
                    self._cd_fallback_node = None
                    self._cd_fallback_xy = None
                    self.get_logger().warn(
                        f'★CLEAR_DETOUR 우회 goal 거부 → 다음다음 노드 '
                        f'{self._cd_target_node}로 폴백 재시도(1회)')
                    self._send_clear_detour()
                    return
                # 폴백까지 소진 → 기존 RETRYING(→소진 시 ESCAPE) 폴백. 관제 확인 필요.
                self._cd_detour_active = False
                self._cd_target_xy = None
                self._cd_target_node = None
                if self.current_state == PatrolState.MOVING_TO_EVENT:
                    # ★EVT_DETOUR(7/22): 이벤트 우회 거부 → 파견 재시도 경로(폴백 노드 없음)
                    self.get_logger().warn('★EVT_DETOUR 우회 goal 거부 → 파견 재시도 경로')
                    self._handle_dispatch_failure()
                    return
                self.get_logger().warn(
                    '★CLEAR_DETOUR 우회 goal 거부 — 우회 실패, 관제 확인 필요 → RETRYING 폴백')
                self.change_state(PatrolState.RETRYING)
                return
            if self.current_state == PatrolState.MOVING_TO_EVENT:
                self._handle_dispatch_failure()
            else:
                self.change_state(PatrolState.RETRYING)
            return
        # ★ESTOP_FIX 1: accepted된 goal 핸들을 보관 (나중에 취소하려면 필요)
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.handle_goal_result)
        # ★ESTOP_HARD(7/9): 발사→accept 사이에 비상정지/수동 진입했으면 이 goal은 유령 — 즉시 취소
        if self.current_state in (PatrolState.EMERGENCY_STOP, PatrolState.MANUAL_CONTROL):
            self.get_logger().warn('비상정지/수동 중 늦게 accept된 goal → 즉시 취소')
            self.cancel_current_goal()

    def handle_goal_result(self, future):
        status = future.result().status
        # ★NAV_REPORT: 마지막 Nav2 goal 결과를 문자열로 보관(관제 표시용, 로직 무관여)
        self._last_goal_status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
            status, f'STATUS_{status}')
        self.goal_in_flight = False
        self.current_goal_handle = None        # ★ESTOP_FIX 2B: 끝난 goal 핸들 정리

        # ★ESTOP_FIX 2B (+Phase 2): 가드 — nav 결과를 '기다리는' 상태가 아니면 결과를 버린다.
        if self.current_state not in (PatrolState.PATROLLING,
                                      PatrolState.MOVING_TO_EVENT,
                                      PatrolState.RETURNING_TO_CHARGER):
            self.get_logger().info(
                f'goal 결과(status={status}) 무시 — 현재 {self.current_state.name}')
            return

        # ★CLEAR_DETOUR: 우회 goal 결과는 반드시 'PATROLLING + _cd_detour_active'에서만 도착
        #   (다른 상태로 이탈했으면 위 가드가 결과를 버리고, run_state_loop가 플래그를 폐기).
        #   성공 → 상태 유지(PATROLLING): _rs_tick이 최근접 노드부터 잔여 구간 이어달리기 재개.
        #   실패 → 기존 RETRYING(→소진 시 ESCAPE) 폴백.
        if (self._cd_detour_active
                and self.current_state in (PatrolState.PATROLLING,
                                           PatrolState.MOVING_TO_EVENT)):
            self._cd_detour_active = False
            detour_node = self._cd_target_node
            self._cd_target_xy = None
            self._cd_target_node = None
            if status == 4:  # SUCCEEDED
                self.retry_count = 0
                self.stuck_retry_count = 0
                self._cd_auto_detour_used = False   # 우회 성공 = 진전 → 자율우회 티켓 리셋
                if self.current_state == PatrolState.MOVING_TO_EVENT:
                    # ★EVT_DETOUR(7/22): 스탠드오프 도착 — 다음 틱 evt_active가 서보
                    #   이벤트 접근을 새 위치에서 재초기화(TURN→정렬→60cm 컷).
                    self.get_logger().info(
                        '★EVT_DETOUR 우회 성공(스탠드오프 도착) → 서보 이벤트 접근 재개')
                else:
                    self.get_logger().info(
                        f'★CLEAR_DETOUR 우회 성공(노드{detour_node} 부근 도착) '
                        f'→ 서보 이어달리기(최근접 노드부터 잔여 구간 재개)')
            else:
                if self.current_state == PatrolState.MOVING_TO_EVENT:
                    self.get_logger().warn(
                        f'★EVT_DETOUR 우회 실패(status={status}) → 파견 재시도 경로')
                    self._handle_dispatch_failure()
                    return
                self.get_logger().warn(
                    f'★CLEAR_DETOUR 우회 실패(status={status}) — 관제 확인 필요 '
                    f'→ RETRYING 폴백(소진 시 ESCAPE)')
                self.change_state(PatrolState.RETRYING)
            return

        if status == 4:  # SUCCEEDED
            self.retry_count = 0
            self.stuck_retry_count = 0
            if self.current_state == PatrolState.MOVING_TO_EVENT:
                # ★HANDOVER 분기: 파견 도착 시 event_type에 따라 갈림
                self.dispatch_retry_count = 0
                # ★CENTER_FALLBACK(7/8): dispatch_target을 None 하기 전에 대상 방향(yaw=θ) 저장.
                #   중앙정렬 중 bearing(실시간 방위) 유실 시 이 방향으로 개루프 회전 폴백 → 물체 정면 응시.
                if self.dispatch_target is not None:
                    _q = self.dispatch_target.pose.orientation
                    self._event_target_yaw = math.atan2(
                        2 * (_q.w * _q.z + _q.x * _q.y),
                        1 - 2 * (_q.y * _q.y + _q.z * _q.z))
                self.dispatch_target = None
                if getattr(self, 'dispatch_event_type', '') == 'HANDOVER':
                    # 인계 지점 도착 -> 그 wp부터 순찰 재개 (PAUSED 안 함)
                    if 0 <= self.dispatch_target_wp < len(self.waypoints):
                        self.current_waypoint_index = self.dispatch_target_wp
                    else:
                        self.get_logger().warn(
                            f'HANDOVER wp 인덱스 이상({self.dispatch_target_wp}) -> 0')
                        self.current_waypoint_index = 0
                    self.get_logger().info(
                        f'HANDOVER 인계 완료 -> wp{self.current_waypoint_index}부터 순찰')
                    self.change_state(PatrolState.PATROLLING)
                elif self._nav2_event():
                    # ★NAV2_EVENT(7/8): Nav2 도착 → 폐루프 중앙정렬 시작(_rs_tick 수행) → 완료 시 PAUSED.
                    self.get_logger().info('파견 현장 Nav2 도착 → 폐루프 중앙정렬 시작')
                    self._event_centering = True
                    self._event_center_t0 = self._rs_now_sec()
                    self._reacq_begin_episode()   # ★REACQ_LOOPFIX: 새 정렬 국면 → 재탐색 기회 1회 복원
                    self._center_bmin = None
                else:
                    # 일반 파견(HELMET/FALL/FIRE): 현장 도착 -> PAUSED (사진+관제 대기)
                    self.get_logger().info('파견 현장 도착 -> PAUSED')
                    self.change_state(PatrolState.PAUSED, reason=self._dispatch_reason())
            elif self.current_state == PatrolState.RETURNING_TO_CHARGER:
                if self.get_parameter('use_dock_executor').value:
                    # ★RTC_PRECISE(7/9): Nav2 도착오차는 느슨(±15cm+) — 그 자리서 바로 DOCK하면
                    #   마커 탐색 기하가 깨져 타임아웃(7/9 실주행 실증). 서보 게이트 ON이면
                    #   기존 '프리도킹 정밀 재접근'(_rs_dock_retry, ±7cm 자가교정) 경유 후 도킹.
                    if self.get_parameter('use_route_servo').value:
                        self.get_logger().info(
                            '충전 복귀 Nav2 도착(느슨) → 프리도킹 서보 정밀 재접근 → 도킹')
                        self._rtc_docking = True
                        self._rs_dock_retry = True
                        self._rs_fresh = False
                        self.change_state(PatrolState.PATROLLING)
                        return
                    # ★RTC_DOCK(7/7): 충전 복귀도 랩 종료와 동일한 마커 정밀도킹 경유.
                    #   pre-dock 도착 → DOCKING(SEARCH→ALIGN→REVERSE) → _on_dock_zone_arrived가
                    #   _rtc_docking 플래그 보고 랩판정 없이 CHARGING 직행.
                    self.get_logger().info('충전 복귀 pre-dock 도착 → DOCKING (마커 정밀도킹, 랩 종료와 동일)')
                    self._rtc_docking = True
                    self.dock_cmd_sent = False
                    self.change_state(PatrolState.DOCKING)
                else:
                    # 게이트 OFF: 기존 동작 — 근처 도착 즉시 CHARGING(단자 연결 대기)
                    self.get_logger().info('충전소 근처 도착 -> CHARGING (충전 단자 연결 대기)')
                    self.charging_start = None   # CHARGING 루프 첫 진입에서 타이머 시작
                    self.change_state(PatrolState.CHARGING)
            else:
                self.get_logger().info(f'Arrived at wp{self.current_waypoint_index}')
                self.change_state(PatrolState.ARRIVED)
        else:
            self.get_logger().warn(f'Goal failed (status={status})')
            if self.current_state == PatrolState.MOVING_TO_EVENT:
                self._handle_dispatch_failure()
            elif self.current_state == PatrolState.RETURNING_TO_CHARGER:
                self.get_logger().warn('충전소 goal 실패 — ESCAPE(탈출) 시도')
                self._enter_escape(PatrolState.RETURNING_TO_CHARGER)
            else:
                self.change_state(PatrolState.RETRYING)

    def _handle_dispatch_failure(self):
        """MOVING_TO_EVENT에서 goal 실패/거부 시 처리."""
        self.dispatch_retry_count += 1
        if self.dispatch_retry_count < self.dispatch_max_retries:
            self.get_logger().warn(
                f'파견 goal 실패 — 재시도 {self.dispatch_retry_count}/'
                f'{self.dispatch_max_retries} (다음 루프에서 재발사)')
        else:
            # ★DISPATCH_FAIL_RESUME(7/8): 파견 접근 실패 = 이벤트 포기하고 '순찰 재개'.
            #   (구: LOW_BATTERY→충전복귀 = 배터리 멀쩡한데 충전소로 가는 오동작. 사용자 지적)
            #   실패한 이벤트 위치는 융합노드가 이미 블랙리스트(120s)라 즉시 재파견 안 됨.
            #   RESUMING = saved_waypoint_index부터 순찰 재개(서보가 최근접 노드부터 이어달리기).
            # ★SETTLE_NEAR(7/10, 사용자 요구): "좌표에 일단 가까이 간 다음, 금지존 위거나 못 가면
            #   그 근처에서 그걸 바라보고 있는 걸로 만족하자."
            #   종전엔 이벤트를 통째로 포기했다(오늘 실증: Goal failed ×5 → 순찰 재개). 그러나 접근
            #   실패의 대부분은 '목표가 인플레이션/금지존 안'이라 Nav2가 abort하는 것이고, 로봇은
            #   이미 대상 근처에 서 있다. 사진만 못 찍고 돌아가는 건 손해다.
            #   → 현 위치에서 대상을 바라보고 도착 마무리(중앙정렬→거리조정→최종정렬→PAUSED)를 돌린다.
            #   대상이 화각 밖이면 재탐색이 훑고, 그래도 없으면 저장된 _event_target_yaw 방향으로 응시.
            if (self.get_parameter('settle_near_on_fail').value
                    and self.dispatch_target is not None):
                self.get_logger().warn(
                    f'파견 {self.dispatch_max_retries}회 실패 — 접근 불가(금지존/인플레이션 추정). '
                    f'★현 위치에서 대상 응시 후 마무리(포기하지 않음)')
                self.cancel_current_goal()
                self._rs_stop()
                self.dispatch_retry_count = 0
                self._event_centering = True
                self._event_center_t0 = self._rs_now_sec()
                self._reacq_begin_episode()
                self._event_final_phase = None   # 중앙정렬부터 순서대로
                return
            self.get_logger().error(
                f'파견 {self.dispatch_max_retries}회 실패 -> 이벤트 포기, 순찰 재개(RESUMING)')
            self.dispatch_target = None
            self.dispatch_retry_count = 0
            self._event_target_yaw = None
            self.change_state(PatrolState.RESUMING)

    def _dispatch_reason(self):
        et = getattr(self, 'dispatch_event_type', 'EVENT')
        return f'EVENT_{et}'

    # ================================================================
    # 입력 핸들러: 사람 명령(set_mode)
    # ================================================================
    def handle_set_mode(self, request, response):
        mode = request.mode
        self.get_logger().info(f'set_mode 수신: {mode}')

        if mode == 'PATROL_START':
            # ★CHARGE_SWAP(7/17, 사용자: "차징인데 35프로 이상이면 교대 그냥 나가는걸로"):
            #   PATROL_START 는 원래 IDLE 에서만 수락 → 배터리 40% 이하로 도킹해 CHARGING 에
            #   들어간 로봇은 서버 교대 신호를 거부했다('IDLE이 아님 (현재 CHARGING)').
            #   두 대가 다 40% 아래면 둘 다 거부 = 순찰 정지(7/15 실측). → CHARGING 이라도
            #   배터리가 충분(기본 35%)하면 출발을 허용한다.
            #   ※ battery_threshold(33%)와 간격이 2%뿐이라, 나가자마자 충전복귀가 걸릴 수 있다.
            #     그 간격을 넓히려면 charging_dispatch_min_pct 를 올릴 것.
            _allow = self.current_state == PatrolState.IDLE
            if (not _allow and self.current_state == PatrolState.CHARGING
                    and bool(self.get_parameter('use_charging_dispatch').value)):
                _minb = float(self.get_parameter('charging_dispatch_min_pct').value)
                if float(self.current_battery) >= _minb:
                    _allow = True
                    self.get_logger().warn(
                        f'★CHARGE_SWAP: CHARGING 이지만 배터리 {self.current_battery:.0f}% ≥ {_minb}% '
                        f'→ 교대 출발 허용(충전 중단)')
                else:
                    self.get_logger().warn(
                        f'★CHARGE_SWAP 거부: 배터리 {self.current_battery:.0f}% < {_minb}% — 충전 우선')
            if _allow:
                # ★DOCK_SIG(7/15 승인): 도크좌표 주입은 '실물이 도크에 있을 때만'.
                #   라이다 뒷벽 시그니처(도킹상태=뒷벽 ~0.18m)로 검사 — 손이동 후 엉뚱한 곳에서
                #   PATROL_START 시 도크좌표를 강제 주입해 눈뜬장님이 되는 사고(오늘 다발) 차단.
                #   scan 미수신(None)이면 옛 동작(주입) 유지 — 링크죽음은 별개 문제.
                _rear = getattr(self, '_rs_rear', None)
                _sig_max = float(self.get_parameter('dock_sig_rear_max').value)
                if _rear is not None and _rear > _sig_max:
                    self.get_logger().warn(
                        f'★DOCK_SIG 불일치: 뒷벽 {_rear:.2f}m > {_sig_max:.2f}m — 실물이 도크에 없음. '
                        f'도크좌표 주입 생략(현 believed 위치로 출발. 위치 어긋났으면 2D Pose 후 재시작)')
                else:
                    self.publish_initialpose()   # ★DRIFT_RESET: 출발 시 도크좌표 주입(도크 시그니처 확인됨)
                self.dock_arrival_count = 0  # ★MAX_LAPS: 새 순찰 시작 → 바퀴 카운트 리셋(이번 run 3바퀴 새로)
                self._handover_sent = False  # ★감사픽스: 새 순찰 = 교대 이력 리셋
                self.charging_start = None   # ★CHARGE_SWAP(7/17): CHARGING 에서 출발 시 충전 세션 종료
                self.current_waypoint_index = 1  # ★LAP_FIX+①: wp1부터 시작. wp0(0.34,0.15)은 undock위치 옆 15cm 측면목표라 RPP가 abort → 건너뜀. (idx=0이면 undock후 첫목표 실패→ESCAPE thrash)
                # ★UNDOCK(게이트): executor면 전진 undock 먼저(빨강칸 ESCAPE 회피), 아니면 옛 LOCALIZING
                if self.get_parameter('use_dock_executor').value:
                    self.undock_cmd_sent = False
                    self.change_state(PatrolState.UNDOCKING)
                    response.message = 'PATROL_START -> UNDOCKING'
                else:
                    self.change_state(PatrolState.LOCALIZING)
                    response.message = 'PATROL_START -> LOCALIZING'
                response.success = True
            else:
                response.success = False
                response.message = f'IDLE이 아님 (현재 {self.current_state.name})'

        elif mode == 'RESUME':   # Play: PAUSED -> 저장된 wp부터 재개
            if self.current_state == PatrolState.PAUSED:
                self.change_state(PatrolState.RESUMING)
                response.success = True
                response.message = 'RESUME -> RESUMING'
            elif self.current_state == PatrolState.EMERGENCY_STOP:
                # ★ESTOP_RESUME(7/10): 비상정지 해제 = 하던 일 마저 하기(사용자 요구).
                #   예전엔 여기서 거부돼 RESET(전체 초기화→IDLE)밖에 길이 없었다 = 임무 전부 소실.
                prev = self._estop_prev
                self._estop_prev = None
                self._estop_release = True   # ★ESTOP_LATCH(7/13): 관제 해제 — 이 핸들러만 래치를 연다
                nxt = self._estop_resume_state(prev)
                self.get_logger().warn(
                    f'★비상정지 해제 → 재개: 직전={prev.name if prev else "?"} → {nxt.name}')
                # ★UNWEDGE(7/10, 사용자 지적): ESTOP은 로봇을 '아무 데서나' 세운다 — 벽 코앞/인플레이션
                #   안일 확률이 높다. 그 자리에서 바로 Nav2 goal을 쏘면 컨트롤러가 유효속도를 못 만들어
                #   '무이동 7s → ESCAPE 강제발동'으로 끼인다(7/10 실주행 로그). 끼어 있으면 먼저 빼낸다.
                #   ESCAPE는 탈출 후 return_state로 스스로 복귀하므로 재개 임무가 보존된다.
                drive_states = (PatrolState.RESUMING, PatrolState.MOVING_TO_EVENT,
                                PatrolState.RETURNING_TO_CHARGER)
                if nxt in drive_states and self._is_wedged():
                    self.get_logger().warn(
                        f'★재개 지점이 끼임(벽/금지존 근접) → 먼저 ESCAPE로 빼낸 뒤 {nxt.name} 복귀')
                    self._enter_escape(nxt)
                    self._estop_release = False   # ★ESTOP_LATCH: 해제창 닫기
                    response.success = True
                    response.message = f'RESUME(ESTOP) -> ESCAPE -> {nxt.name}'
                    return response
                self.change_state(nxt)
                self._estop_release = False       # ★ESTOP_LATCH: 해제창 닫기
                response.success = True
                response.message = f'RESUME(ESTOP) -> {nxt.name}'
            else:
                response.success = False
                response.message = f'PAUSED가 아님 (현재 {self.current_state.name})'

        elif mode == 'MANUAL_ENTER':
            self.cancel_current_goal()          # ★ESTOP_FIX 2A: 자동주행 goal 끊고 수동으로
            self.save_waypoint_index()          # ★ESTOP_FIX 4: RESUME용 현재 wp 저장
            self.change_state(PatrolState.MANUAL_CONTROL)
            response.success = True
            response.message = 'MANUAL_CONTROL'

        elif mode == 'MANUAL_EXIT':
            if self.current_state == PatrolState.MANUAL_CONTROL:
                self.change_state(PatrolState.PAUSED, reason='MANUAL_DONE')
                response.success = True
                response.message = 'MANUAL_EXIT -> PAUSED(MANUAL_DONE)'
            else:
                response.success = False
                response.message = 'MANUAL_CONTROL이 아님'

        elif mode == 'EMERGENCY_STOP':
            # ★ESTOP_RESUME(7/10): 하던 일을 기억한다(연타해도 첫 값 유지 — EMERGENCY_STOP 자신은 저장 금지).
            if self.current_state != PatrolState.EMERGENCY_STOP:
                self._estop_prev = self.current_state
                self.save_waypoint_index()      # Nav2 경로용 wp도 저장(서보는 _rs_full/_rs_leg가 보존됨)
            self.cancel_current_goal()          # ★ESTOP_FIX 2A: 굴러가던 goal 끊기 (핵심)
            # ★ESTOP_CANCEL(7/10): 도킹 중엔 바퀴를 Pi(pi_dock)가 굴린다. PC의 0속도는 같은 /cmd_vel을
            #   두고 경쟁할 뿐이라 안 멈췄다(실측 31.8s / 3분55초). Pi가 CANCEL을 실제로 처리하도록 픽스됨.
            self.dock_cmd_pub.publish(String(data='CANCEL'))
            self.dock_cmd_sent = False          # 재개 시 DOCK 재발행 (Pi는 CANCEL로 IDLE 복귀)
            self.undock_cmd_sent = False
            self._rs_stop()                                    # ★감사픽스: 서보 즉시 0속도
            self.change_state(PatrolState.EMERGENCY_STOP)
            response.success = True
            response.message = f'EMERGENCY_STOP (직전={self._estop_prev.name if self._estop_prev else "-"})'

        elif mode == 'RESET':
            # ★RESET_ANY(7/7, 시나리오 6-7): 어떤 상태에서든 전체 초기화 → IDLE.
            #   (구버전은 EMERGENCY_STOP에서만 허용 — 관제 '초기화' 버튼 시나리오와 불일치)
            self._estop_release = True   # ★ESTOP_LATCH(7/13): RESET 도 관제 해제 경로
            self.cancel_current_goal()
            self.dock_cmd_pub.publish(String(data='CANCEL'))   # ★감사픽스: Pi 도킹/언도킹 모션 중단
            self._rs_stop()
            self._rs_reset()
            self._cd_detour_active = False
            self._cd_backoff_t0 = None                     # ★DETOUR_BACKOFF: 후진도 초기화
            self._cd_target_xy = None
            self._cd_target_node = None
            self._cd_fallback_node = None
            self._cd_fallback_xy = None
            self._rtc_docking = False
            self._estop_prev = None       # ★ESTOP_RESUME: 전체 초기화 = 이어받을 임무 없음
            self._dock_done_msg = None    # 낡은 완료신호 잔재 제거(재개 시 가짜 성공 방지)
            self._rs_dock_retry = False   # ★RTC_PRECISE: 정밀 재접근 티켓도 초기화
            self._handover_sent = False
            self._batt_low_since = None
            self.dispatch_target = None
            self._idle_dispatch_pending = False
            self.dock_cmd_sent = False
            self.undock_cmd_sent = False
            self.charging_start = None
            self.change_state(PatrolState.IDLE)
            self._estop_release = False   # ★ESTOP_LATCH: 해제창 닫기
            response.success = True
            response.message = 'RESET -> IDLE (전체 초기화)'

        elif mode == 'RETURN_TO_CHARGER':
            self.cancel_current_goal()      # ★FIX: 진행 중 순찰 goal 끊기 (안 끊으면 그 wp를 충전소 도착으로 오인 → 엉뚱한 자리서 CHARGING)
            self.save_waypoint_index()
            # ★RTC_NAV2(7/8): RTC는 어디서 걸릴지 모르므로 Nav2가 현재위치→프리도킹 최단경로로 복귀.
            #   금지존/동적장애물은 global+local costmap의 keepout_filter/obstacle_layer/collision_monitor가
            #   회피(경로계획+실시간). 프리도킹 도착 → DOCKING(마커 후진)으로 (0,0) 정밀진입 → CHARGING.
            #   goal 도착오차는 느슨(patrol_*_tolerance) — 초정밀 도킹은 마커가 담당. 어제 ESCAPE 루프
            #   원인이던 dock 초정밀오차(yaw14° < AMCL떨림18°) 교착을 회피. (구 서보경유 방식 폐기:
            #   시작점 근처서 RTC 시 프리도킹까지 거의 한바퀴 돌던 문제)
            self._rtc_docking = True
            # ★RTC_NEARDOCK(7/10, 실주행 실증): 로봇이 이미 도킹존 빨강칸 안/근처면 Nav2를 부르면 안 된다.
            #   RPP가 첫 틱부터 'detected collision ahead' → 'Controller patience exceeded' → abort.
            #   그 자리서 무한루프(무이동 7s → ESCAPE → 재시도)가 된다.
            #   실측: 로봇 (0.33,-0.02), 목표 노드13 (0.55,0.01) — 22cm 앞인데 한 발짝도 못 감.
            #   → Nav2 생략하고 서보 프리도킹 직행(_rs_dock_retry). 서보는 도킹 계열이라 이동 가드도 면제.
            p = self._rs_pose()
            near = False
            if p is not None and self.charging_station is not None:
                # ★버그(7/10 실주행 즉사): charging_station 은 yaml dict {'x','y','yaw_z'} 다.
                #   [0]/[1] 인덱싱 → KeyError: 0 → 노드 죽음. RTC 누를 때만 실행돼 빌드·문법검사를 통과했다.
                d = math.hypot(p[0] - float(self.charging_station['x']),
                               p[1] - float(self.charging_station['y']))
                near = d < float(self.get_parameter('rtc_direct_radius').value)
            if near:
                self.get_logger().warn(
                    f'★RTC: 도킹존 근처({d:.2f}m) — Nav2 생략, 서보로 프리도킹 직행'
                    f'(빨강칸에선 Nav2가 첫 틱부터 abort)')
                self._rs_reset()
                self._rs_dock_retry = True
                self._rs_fresh = False
                self.change_state(PatrolState.PATROLLING)   # 서보가 _rs_build_lap 에서 직행 레그 구성
                response.success = True
                response.message = 'RETURNING_TO_CHARGER (도킹존 근처 — 서보 직행)'
                return response
            self.change_state(PatrolState.RETURNING_TO_CHARGER)
            response.success = True
            response.message = 'RETURNING_TO_CHARGER (Nav2 최단경로 복귀)'

        else:
            response.success = False
            response.message = f'알 수 없는 mode: {mode}'

        return response

    # ================================================================
    # 입력 핸들러: 파견(dispatch) — 안전모 접근촬영 / HANDOVER
    # ================================================================
    def _apply_dispatch_standoff(self, target, event_type):
        """★DISPATCH_STANDOFF(7/10): 파견 target 은 '대상 원위치'다 — 그 위로 가면 안 된다.
        로봇→대상 직선 위에서 event_stop_range(0.6m) 만큼 당긴 지점을 실제 goal 로 삼는다.

        왜 여기(patrol)인가: 종전엔 fusion 이 미리 60cm 를 빼서 보냈고, 서버(글로벌캠) 파견은
        대상 위치를 그대로 보내 **두 경로의 의미가 달랐다**(서버 파견은 대상 위로 goal → Nav2 가
        인플레이션에 걸려 'detected collision ahead → abort'). 서비스 스키마(v1.0)에 '원본/보정본'
        구분 필드가 없으므로, 규약을 "target 은 언제나 대상 원위치"로 통일하고 standoff 는
        여기 한 곳에서만 적용한다(fusion 은 send_raw_target=True 로 원위치를 보낸다).

        대상이 standoff 보다 가까우면 r_eff<0 → goal 이 로봇 뒤 → 기존 BACKOFF 가 후진(의도된 동작).
        HANDOVER 는 '인계 wp 좌표'라 대상이 아니다 → 적용 제외.
        """
        if (event_type == 'HANDOVER'
                or not self.get_parameter('dispatch_apply_standoff').value):
            return target
        p = self._rs_pose()
        if p is None:
            self.get_logger().warn('★standoff: 로봇 pose 없음 — 원본 좌표 사용')
            return target
        tx, ty = target.pose.position.x, target.pose.position.y
        # ★EARLY_STOP(7/10): 대상 '원위치'를 기억한다 — 접근 중 남은 거리를 재려면 필요.
        #   (dispatch_target 은 standoff 적용 후 goal 이라 대상 위치가 아니다.)
        self._event_target_raw = (tx, ty)
        dx, dy = tx - p[0], ty - p[1]
        r = math.hypot(dx, dy)
        if r < 1e-3:
            return target
        # ★RAW_GOAL(7/10): raw 모드면 standoff 를 빼지 않는다 — 받은 좌표가 곧 goal.
        raw_mode = bool(self.get_parameter('use_raw_goal_event').value)
        so = 0.0 if raw_mode else float(self.get_parameter('event_stop_range').value)
        r_eff = r - so
        out = PoseStamped()
        out.header.frame_id = target.header.frame_id or 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        # ★GOAL_SAFE(7/10, 실주행 실증): standoff 지점이 금지존 코앞일 수 있다.
        #   실측: 대상(-0.13,1.19) → goal(0.31,0.79) 이 금지존 경계에서 3.4cm.
        #   로봇 반경 0.10m 이니 '도착 = 발자국 침범'. Nav2 는 goal 이 코스트맵상 유효하면 몰고 간다
        #   (하드가드는 서보 속도만 검사하므로 Nav2 경로에는 서 있지도 않다).
        #   → goal 이 금지존+반경+여유 안이면 로봇 쪽으로 5cm 씩 당겨 안전 지점을 찾는다.
        #   못 찾으면(대상이 금지존 안) 로봇 현위치를 goal 로 → 그 자리서 응시·마무리(settle_near).
        gx = p[0] + r_eff * dx / r
        gy = p[1] + r_eff * dy / r
        clr = float(self.get_parameter('goal_keepout_clearance').value)
        blk = int(self.get_parameter('escape_cost_block').value)

        def _bad(x, y):
            """goal 로 쓰기에 부적합한가. 기본=금지존 근접.
            ★RAW_GOAL: raw 모드에선 전역 코스트맵의 장애물(인플레이션 포함)도 함께 본다.
            대상 좌표는 대상 자신이 서 있는 자리라 그대로 두면 Nav2 가 abort 하기 때문."""
            if self._mask_near(x, y, clr):
                return True
            if raw_mode:
                c = self._g_cost_at(x, y)
                if c is not None and c >= blk:
                    return True
            return False

        if self._km is not None or raw_mode:
            gx0, gy0 = gx, gy
            if _bad(gx, gy):
                # ★GOAL_NEAREST(7/17, 사용자: "인플 안에 있으면 인플 밖 가장 가까운 데로 목적지를 지정해"):
                #   종전엔 '로봇 쪽으로만' 5cm 씩 후퇴 → 대상이 금지존 옆이면 옆으로 몇 cm 만 비켜도
                #   되는데 뒤로 멀리 물러났다(사진 거리 손해 + 실패 확률↑).
                #   → 원래 goal 을 중심으로 동심원을 넓혀가며 훑어 '가장 가까운 유효점'을 쓴다.
                #     _bad() 는 금지존+clearance(인플레이션 포함하도록 0.15) ∪ 전역 코스트맵 장애물.
                found = None
                if bool(self.get_parameter('use_goal_nearest_outside').value):
                    step = 0.05
                    for i in range(1, 21):            # 5cm ~ 1.0m
                        rad = step * i
                        n = max(8, int(2 * math.pi * rad / 0.05))
                        for k in range(n):
                            a = 2 * math.pi * k / n
                            cx_, cy_ = gx0 + rad * math.cos(a), gy0 + rad * math.sin(a)
                            if not _bad(cx_, cy_):
                                found = (cx_, cy_, rad)
                                break
                        if found:
                            break
                if found:
                    gx, gy, _rad = found
                    self.get_logger().warn(
                        f'★GOAL_NEAREST: goal 이 금지존/인플레이션 안 → 밖 최근접점으로 이동 '
                        f'({_rad:.2f}m) → map({gx:.2f},{gy:.2f})')
                else:
                    # 폴백(옛 동작): 로봇 쪽으로 5cm 씩 후퇴
                    back = 0.0
                    while _bad(gx, gy) and r_eff - back > 0.0:
                        back += 0.05
                        gx = p[0] + (r_eff - back) * dx / r
                        gy = p[1] + (r_eff - back) * dy / r
                    if back > 0.0:
                        _why = '금지존/장애물' if raw_mode else f'금지존 {clr:.2f}m'
                        self.get_logger().warn(
                            f'★GOAL_SAFE: goal 이 {_why} 이내 → {back:.2f}m 뒤로 당김 '
                            f'→ map({gx:.2f},{gy:.2f}) (대상까지 {r - r_eff + back:.2f}m)')
            if _bad(gx, gy):
                self.get_logger().error(
                    '★GOAL_SAFE: 안전한 접근점 없음(대상이 금지존 안?) — 현위치 유지, 응시만')
                gx, gy = p[0], p[1]
        out.pose.position.x = gx
        out.pose.position.y = gy
        # 도착 후 대상을 바라보는 방향(FACEYAW).
        # ★GOAL_NEAREST(7/17): goal 이 옆으로 비켜났으면 '로봇→대상' 방향이 아니라
        #   '옮긴 goal→대상' 방향을 봐야 대상이 화면에 잡힌다(옆으로 옮기고 옛 방향을 보면 헛방).
        _gdx, _gdy = tx - gx, ty - gy
        theta = math.atan2(_gdy, _gdx) if math.hypot(_gdx, _gdy) > 1e-3 else math.atan2(dy, dx)
        out.pose.orientation.z = math.sin(theta / 2.0)
        out.pose.orientation.w = math.cos(theta / 2.0)
        _tail = '(로봇 뒤 — BACKOFF 후진)' if r_eff < 0 else f'(대상 {so:.2f}m 앞)'
        self.get_logger().info(
            f'★standoff 적용({event_type}): 대상 map({tx:.2f},{ty:.2f}) 거리 {r:.2f}m → '
            f'goal map({out.pose.position.x:.2f},{out.pose.position.y:.2f}) {_tail}')
        return out

    def _nav2_event(self):
        """★HYBRID_FALLBACK(7/13): 이 이벤트를 Nav2 로 접근하는가.
        기본은 서보(use_nav2_event_approach=False)지만, 서보 직선이 멀리서 막히면
        _event_force_nav2 로 이 이벤트만 Nav2 우회. 새 파견 수락 시 리셋."""
        return (bool(self.get_parameter('use_nav2_event_approach').value)
                or self._event_force_nav2)

    def handle_dispatch(self, request, response):
        # ★REFINE(7/7): 접근 중(MOVING_TO_EVENT) 같은 이벤트 타입 재호출 = 목표 좌표 재보정.
        #   회전/화각 가장자리에서 계산된 부정확한 첫 좌표를, 접근하며 정면 재관측한
        #   좌표로 갈아끼움. 서보(_rs_tick)는 dispatch_target을 매 틱 읽으므로 즉시 반영.
        if (self.current_state == PatrolState.MOVING_TO_EVENT
                and self.dispatch_target is not None
                and (request.event_type or 'EVENT') == self.dispatch_event_type):
            old = self.dispatch_target.pose.position
            new = request.target.pose.position
            moved = math.hypot(new.x - old.x, new.y - old.y)
            if moved > 1.0:
                # ★감사픽스(7/7): 1m 이상 떨어진 좌표=재보정이 아니라 별도 신규 이벤트
                #   → 조용한 목표 스왑 방지, 현재 임무 우선(거부. 쿨다운 후 재파견됨)
                response.accepted = False
                response.message = '접근 중 별도 이벤트 — 현재 임무 우선(거부)'
                return response
            self.dispatch_target = self._apply_dispatch_standoff(
                request.target, request.event_type or 'EVENT')
            if self.get_parameter('use_route_servo').value:
                # 목표가 크게 움직였고 조준/주행 단계면 재조준 (FACEYAW/BACKOFF/SETTLE은 불간섭)
                if moved > 0.15 and self._rs_evt and self._rs_phase in ('TURN', 'DRIVE'):
                    p = self._rs_pose()
                    if p is not None and math.hypot(new.x - p[0], new.y - p[1]) < 0.12:
                        # ★감사픽스(ARRIVE_SIZE): 새 목표≈현위치 — TURN 조준은 atan2 노이즈로
                        #   제자리 스핀 위험 → 조준 생략하고 즉시 응시(FACEYAW) 정지
                        self._rs_enter_settle('FACEYAW')
                    else:
                        self._rs_enter_turn()
            else:
                if moved > 0.15:
                    self.cancel_current_goal()   # Nav2 경로: 상태루프가 새 목표로 재발사
            self.get_logger().info(
                f'★REFINE 목표 재보정: map({new.x:.2f},{new.y:.2f}) (이동 {moved:.2f}m)')
            response.accepted = True
            response.message = 'TARGET_REFINED'
            return response
        # ★감사픽스(7/7): 충전 복귀(RTC) 주행 중엔 파견 거부 — 저배터리/복귀가 임무보다 우선.
        #   (RTC는 상태가 PATROLLING이라 아래 게이트를 통과해버리는 구멍 실측)
        if self._rtc_docking:
            response.accepted = False
            response.message = '파견 불가 — 충전 복귀 중(RTC)'
            return response
        # ★IDLE_DISPATCH(7/7, 시나리오 5-6): 대기(주차) 중 새 작업 요청 수락.
        #   IDLE=보통 도크 위 → 언도크(UNDOCKING) 먼저 하고 임무(MOVING_TO_EVENT)로.
        #   CHARGING은 계속 거부(충전 우선 — 사용자 확정 7/10). 아래 일반 게이트에서 걸러진다.
        # ★IDLE_BATT_GATE(7/10): IDLE 파견에 배터리 하한(dispatch_min_battery)을 건다.
        #   종전엔 배터리를 전혀 안 봐서, 20%로 주차된 로봇도 출동했다 → 나가자마자 LOW_BATTERY(33%)로
        #   되돌아오거나 맵 한가운데서 방전. 대기 상태라고 출동 가능한 게 아니다.
        if self.current_state == PatrolState.IDLE:
            minb = float(self.get_parameter('dispatch_min_battery').value)
            if self.current_battery < minb:
                response.accepted = False
                response.message = (f'파견 불가 — 배터리 {self.current_battery:.0f}% < {minb:.0f}% '
                                    f'(충전 우선, IDLE)')
                self.get_logger().warn(response.message)
                return response
            self.dispatch_target = request.target
            self.dispatch_target_wp = request.target_wp_index
            self.dispatch_event_type = request.event_type or 'EVENT'
            self.active_type_pub.publish(String(data=self.dispatch_event_type))   # ★MISSION_TYPE
            self.dispatch_retry_count = 0
            self._event_force_nav2 = False   # ★HYBRID_FALLBACK: 새 이벤트는 서보부터
            self._snap_stop = False          # ★SNAP_STOP: 새 이벤트는 신호정지 티켓 초기화
            # ★PHOTO_SPOT(7/14): 촬영지점 좌표·응시방향 기억 + 이전 이벤트 잔재(stale raw) 교체.
            #   (IDLE 경로는 standoff 를 안 타서 _event_target_raw 가 이전 이벤트 값으로 남던 버그)
            _q = request.target.pose.orientation
            # orientation 미기입(기본 quaternion)이면 힌트 없음 — 바로 회전 탐색(사용자: "돌면서 발견")
            self._event_face_yaw = (math.atan2(2 * (_q.w * _q.z + _q.x * _q.y),
                                               1 - 2 * (_q.y * _q.y + _q.z * _q.z))
                                    if abs(_q.z) > 1e-3 or abs(_q.x) > 1e-3 or abs(_q.y) > 1e-3
                                    else None)
            self._event_target_raw = (request.target.pose.position.x,
                                      request.target.pose.position.y)
            self._idle_dispatch_pending = True
            self.undock_cmd_sent = False
            self.get_logger().info(
                f'대기 중 파견 수락: {self.dispatch_event_type} '
                f'(배터리 {self.current_battery:.0f}%) → 언도크 후 출동')
            # ★LOC_GATE(7/14, 사용자 승인): 도크 파견도 PATROL_START 처럼 (0,0,0) 재주입 후 출동.
            #   Pi 재부팅 직후 '위치추정 없는 출동'은 가드가 엉뚱한 셀을 검사하는 눈뜬장님 주행
            #   (14:42 금지존 관통 실측). 전제 동일: IDLE=도크 테이프 위.
            self.publish_initialpose()
            self.change_state(PatrolState.UNDOCKING)
            response.accepted = True
            response.message = 'UNDOCK_THEN_MOVING_TO_EVENT'
            return response
        if self.current_state not in (PatrolState.PATROLLING, PatrolState.ARRIVED):
            response.accepted = False
            response.message = f'파견 불가 상태 ({self.current_state.name})'
            return response
        # ★DOCK_ZONE_NODISPATCH(7/16 → 7/16 수정, 사용자 "니 상태가 도킹일때만 막으라고"):
        #   도킹 상태(DOCKING)는 이미 위 상태게이트(PATROLLING/ARRIVED만 수락)에서 거부된다.
        #   RTC 도킹도 위(_rtc_docking)에서 거부. 도킹 '재접근'(_rs_dock_retry, 상태는 PATROLLING)
        #   중엔 이벤트가 도킹보다 우선이므로 수락한다 — 여기서 별도로 막지 않는다.
        #   (이전 근접반경(_near_dock) 차단은 순찰 중 도크 옆만 지나도 helmet을 씹어서 제거함.)

        self.cancel_current_goal()      # ★DISPATCH FIX: 진행 중 순찰 goal 끊기 (파견 goal이 나가게)
        # ★PREFACE(7/9): 발견 즉시 정지 → 이벤트 정면 응시 → 그다음 접근(사용자 요구 순서).
        #   _rs_tick이 bbox 폐루프(/event/bearing)로 대상을 화면 중앙에 놓은 뒤 접근을 시작한다.
        self._rs_stop()   # 즉시 0속도(선응시 플래그는 change_state의 MOVING_TO_EVENT 진입 공통부가 세팅)
        self.save_waypoint_index()
        self.dispatch_target = self._apply_dispatch_standoff(
            request.target, request.event_type or 'EVENT')
        # ★PHOTO_SPOT(7/14): 파견 goal 응시방향 기억(도착 후 제자리 탐색 조준용)
        _q = request.target.pose.orientation
        # orientation 미기입이면 힌트 없음 — 바로 회전 탐색(사용자: "돌면서 발견")
        self._event_face_yaw = (math.atan2(2 * (_q.w * _q.z + _q.x * _q.y),
                                           1 - 2 * (_q.y * _q.y + _q.z * _q.z))
                                if abs(_q.z) > 1e-3 or abs(_q.x) > 1e-3 or abs(_q.y) > 1e-3
                                else None)
        self.dispatch_target_wp = request.target_wp_index   # ★HANDOVER: 인계 wp 저장
        self.dispatch_event_type = request.event_type or 'EVENT'
        self.active_type_pub.publish(String(data=self.dispatch_event_type))   # ★MISSION_TYPE
        self.dispatch_retry_count = 0
        self._event_force_nav2 = False   # ★HYBRID_FALLBACK: 새 이벤트는 서보부터
        self._snap_stop = False          # ★SNAP_STOP: 새 이벤트는 신호정지 티켓 초기화
        # ★RESUME_NAV2SAVE(7/9): '발견 당시 가던 노드' 저장을 파견 수락 시점으로 이동.
        #   기존 저장 코드(_rs_tick의 서보 이벤트 경로)는 use_nav2_event_approach=True(기본)면
        #   한 번도 실행되지 않아 _rs_resume_node가 항상 None → RESUME이 최근접 노드부터
        #   재개 → 순찰 후반 이벤트면 곧 종점 도달 → 도킹으로 가버림(7/9 실주행 증상).
        #   여기서 저장하면 서보/Nav2 어느 접근 모드든 발견 당시 노드부터 이어달리기가 된다.
        if self._rs_full is not None and self._rs_leg + 1 < len(self._rs_full):
            self._rs_resume_node = self._rs_full[self._rs_leg + 1]
            self.get_logger().info(
                f'파견 시점 순찰노드 저장: 재개 시 노드{self._rs_resume_node}부터 이어달리기')
        self.change_state(PatrolState.MOVING_TO_EVENT)
        self.get_logger().info(f'파견 수락: {self.dispatch_event_type} -> MOVING_TO_EVENT')
        response.accepted = True
        response.message = 'MOVING_TO_EVENT'
        return response

    # ================================================================
    # 입력 핸들러: AI 판정(obstacle_event)
    # ================================================================
    def handle_obstacle_event(self, msg):
        # ★EMERGENCY_PATROL: 순찰 중 서버 YOLO가 직접 위험을 발견(EMERGENCY 판정, 시나리오
        #   3-5/4-3)하면 즉시 정지 + PAUSED(EVENT_*) 촬영 대기. 스코프=PATROLLING만
        #   (OBSTACLE_WAITING 분기는 아래 기존 그대로, CLEAR는 순찰 중 무시=현행 유지).
        if (self.current_state == PatrolState.PATROLLING
                and msg.verdict == 'EMERGENCY'
                and self.get_parameter('accept_emergency_in_patrol').value):
            self.get_logger().warn(
                f'순찰 중 EMERGENCY({msg.type or "UNKNOWN"}) 수신 → 즉시 정지 + PAUSED')
            self._rs_stop()             # 서보 주행 즉시 정지(서보 휴면 중이면 0속도 1회 발행=무해)
            self._rs_reset()            # RESUME 복귀 시 최근접 노드부터 이어달리기(_rs_fresh 유지)
            self.cancel_current_goal()  # Nav2 goal(비서보 순찰/CLEAR_DETOUR 우회) 진행 중이면 취소
            if self._cd_detour_active:  # ★CLEAR_DETOUR 우회 중이면 즉시 폐기(하단 정리와 이중 안전)
                self._cd_detour_active = False
                self._cd_backoff_t0 = None                 # ★DETOUR_BACKOFF: 후진도 폐기
                self._cd_target_xy = None
                self._cd_target_node = None
            self.save_waypoint_index()
            self.change_state(PatrolState.PAUSED, reason=f'EVENT_{msg.type or "UNKNOWN"}')
            return
        if self.current_state != PatrolState.OBSTACLE_WAITING:
            return
        if msg.verdict == 'EMERGENCY':
            self.save_waypoint_index()
            self.change_state(PatrolState.PAUSED, reason=f'EVENT_{msg.type or "UNKNOWN"}')
            self.get_logger().info(f'EMERGENCY({msg.type}) -> PAUSED')
        elif msg.verdict == 'CLEAR':
            # ★CLEAR_DETOUR: 서보+게이트 ON이고 막힌 구간 목표가 기억돼 있으면 즉시 복귀 대신
            #   '우회 모드' — 도착노드로 Nav2 goal 1회. 상태는 PATROLLING(관제 표시 일관),
            #   주행만 Nav2가 담당(_rs_tick은 _cd_detour_active 동안 자동 휴면).
            if (self.get_parameter('use_route_servo').value
                    and self.get_parameter('use_clear_detour').value
                    and self._cd_target_xy is not None):
                self.get_logger().info(
                    f'CLEAR -> ★CLEAR_DETOUR 우회 모드: 막힌 구간 도착노드'
                    f'{self._cd_target_node}로 Nav2 우회 시도')
                self._cd_auto_detour_used = False   # 서버가 응답함 = 새 국면 → 자율우회 티켓 리셋
                self._start_clear_detour()
                return
            self.get_logger().info('CLEAR -> 순찰 계속 (Nav2 우회)')
            self.change_state(PatrolState.PATROLLING)

    def _start_clear_detour(self, return_state=PatrolState.PATROLLING):
        """★CLEAR_DETOUR: 우회 모드 시작(공통) — CLEAR 판정 수신 / verdict 무응답 자율우회 겸용.
        상태는 return_state(랩=PATROLLING, ★EVT_DETOUR(7/22)=MOVING_TO_EVENT 유지 — 파견
        맥락 보존), 주행만 Nav2가 담당(서보는 플래그로 휴면)."""
        self._cd_detour_active = True
        self.obstacle_wait_start = None
        if self.current_state != return_state:
            self.change_state(return_state)
        # ★DETOUR_BACKOFF(7/22, 사용자 요청): 정지점이 장애물 코앞(전방 < RS_FRONT_SLOW)이면
        #   Nav2 가 인플레이션 한복판에서 출발해 경로거부/긁힘 위험 — 후방 라이다∪코스트맵
        #   가드 후진으로 간격 확보 후 goal 발사(_rs_tick 이 수행). 간격 넉넉하면 즉시 발사.
        if self._rs_front is not None and self._rs_front < RS_FRONT_SLOW:
            self._cd_backoff_t0 = self._rs_now_sec()
            self.get_logger().info(
                f'★DETOUR_BACKOFF: 전방 {self._rs_front:.2f}m — 후진 간격확보 후 우회 발사')
        else:
            self._send_clear_detour()

    def _send_clear_detour(self):
        """★CLEAR_DETOUR: 막힌 구간의 도착노드 좌표로 NavigateToPose 1회 전송.
        도착 yaw = 현재위치→노드 방향(도착 후 다음 구간 조준은 서보 TURN이 다시 함).
        기존 send_nav_goal/handle_goal_response/handle_goal_result 인프라 재사용.
        Nav2 서버 미준비로 유실되면 run_state_loop(PATROLLING)에서 재발사."""
        tx, ty = self._cd_target_xy
        p = self._rs_pose()
        if p is None:
            p = self._map_pose
        yaw = math.atan2(ty - p[1], tx - p[0]) if p is not None else 0.0
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=float(tx), y=float(ty), z=0.0)
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        # 일반 wp와 동일한 느슨한 도착오차(노드 정밀도는 이후 서보 이어달리기가 확보)
        self.set_goal_tolerance(
            self.get_parameter('patrol_xy_tolerance').value,
            self.get_parameter('patrol_yaw_tolerance').value)
        self.send_nav_goal(pose)

    # ================================================================
    # 입력 핸들러: 라이다 정지 신호(collision_monitor_state)
    # ================================================================
    def handle_collision_state(self, msg):
        """★OBSTACLE_BRIDGE: collision_monitor 정지 신호를 추적만 한다(판단은 run_state_loop).
        라이다(base_footprint, /scan) 기반이라 AMCL 떨림/2D Pose 틀어짐에 영향받지 않음.
        상시발행/변할때만발행 둘 다 견디려고 STOP·비STOP '연속 시작 시각'을 따로 기억한다."""
        now = self.get_clock().now()
        if msg.action_type == CollisionMonitorState.STOP:
            if self.cm_stop_since is None:
                self.cm_stop_since = now      # STOP 연속 시작
            self.cm_clear_since = None
        else:
            if self.cm_clear_since is None:
                self.cm_clear_since = now     # 뚫림 연속 시작
            self.cm_stop_since = None

    # ================================================================
    # ★DOCK_FIX — 도킹존(0,0) 실제 도착 판정 + 도착 처리
    # ================================================================
    def _on_dock_done(self, msg):
        """★UNDOCK/DOCK: Pi 실행노드 완료신호 수신 저장 (UNDOCKING/DOCKING에서 소비)."""
        # ★ESTOP_RESUME(7/10): 비상정지 중 뒤늦게 도착한 완료신호는 버린다.
        #   저장해두면 RESUME 순간 소비되어 '도킹 성공'으로 둔갑한다(실제론 취소됨 → 가짜 CHARGING).
        if self.current_state == PatrolState.EMERGENCY_STOP:
            self.get_logger().warn(f'Pi dock_done={msg.data} — 비상정지 중이라 폐기')
            return
        self._dock_done_msg = msg.data
        self.get_logger().info(f'Pi dock_done 수신: {msg.data}')

    def _post_dock_state(self):
        """도킹존(0,0) 출발 시 다음 상태:
        use_dock_executor=True → UNDOCKING(Pi 전진탈출 후 순찰),
        False → PATROLLING(옛 동작 그대로, 내일 순찰 무영향)."""
        if self.get_parameter('use_dock_executor').value:
            self.undock_cmd_sent = False
            return PatrolState.UNDOCKING
        return PatrolState.PATROLLING

    def _at_dock_zone(self):
        """로봇 실제 pose가 도킹존(charging_station=0,0) 반경 이내인가?
        = 후진도킹으로 pre-dock(0.4,0)에서 (0,0)까지 실제 진입 완료했는지 판정.
        map pose(AMCL) 우선, 없으면 odom 사용."""
        if self.charging_station is None:
            return False
        if self._map_pose is not None:
            px, py = self._map_pose[0], self._map_pose[1]
        elif self._odom is not None:
            px, py = self._odom[0], self._odom[1]
        else:
            return False
        dx = float(self.charging_station['x']) - px
        dy = float(self.charging_station['y']) - py
        return math.hypot(dx, dy) <= self.get_parameter('dock_zone_radius').value

    def _near_dock(self, radius):
        """★DOCK_ZONE_NODISPATCH(7/16): 로봇 pose가 도크(charging_station)에서 radius(m) 이내인가?
        도킹 접근/도크존 판정용(파견 거부). radius<=0 이면 항상 False(기능 끔)."""
        if radius <= 0.0 or self.charging_station is None:
            return False
        if self._map_pose is not None:
            px, py = self._map_pose[0], self._map_pose[1]
        elif self._odom is not None:
            px, py = self._odom[0], self._odom[1]
        else:
            return False
        dx = float(self.charging_station['x']) - px
        dy = float(self.charging_station['y']) - py
        return math.hypot(dx, dy) <= radius

    def _rtc_target_wp(self):
        """★RTC_DOCK: 충전 복귀 접근 목표. 정밀도킹 게이트 ON이면 랩 종료와 동일하게
        pre-dock(마지막 wp)으로 접근(→DOCKING이 마커 정렬+후진으로 (0,0) 진입),
        OFF면 기존대로 충전소(0,0) 직접."""
        if self.get_parameter('use_dock_executor').value and self.waypoints:
            return self.waypoints[-1]
        return self.charging_station

    def _on_dock_zone_arrived(self):
        """★DOCK_FIX: 후진도킹으로 실제 도킹존(0,0) 진입이 확인된 뒤에만 호출.
        한 바퀴 완료 처리: initialpose 리셋 + 도착카운트 + 배터리 handover/충전/DWELL 판정.
        (기존 ARRIVED의 is_dock 분기 로직을 그대로 이관 — pre-dock이 아니라 진짜 (0,0) 도착 시점에 수행)"""
        # ★RTC_DOCK(7/7): 충전 복귀 경유 도킹 = 랩 완료 아님 → 카운트/handover/DWELL 판정
        #   전부 건너뛰고 CHARGING 직행 (기존 RETURNING_TO_CHARGER→CHARGING 동작과 등가).
        if getattr(self, '_rtc_docking', False):
            self._rtc_docking = False
            self.publish_initialpose_if_docked('RTC도킹')   # ★픽스B: 시그니처 확인 후에만 드리프트 리셋
            # ★SERVER_BATTERY(7/17 사용자 확정): "우린 도착하면 무조건 하도버 날려.
            #   하도버 했는데 배터리가 없어서 못 나가는 건 서버가 판단할 일이야."
            #   → 도킹 도착 = 무조건 handover_request 발행 + IDLE 대기. 배터리로 자기를 잠그지 않는다.
            #   ※ CHARGING 상태는 충전을 '시키는' 게 아니라 라벨 + PATROL_START 차단일 뿐이고,
            #     충전은 단자가 꽂히면 하드웨어로 된다 → IDLE 로 대기해도 충전은 똑같이 진행된다.
            #   ※ 서버는 robot_status 의 배터리를 보고 누구를 내보낼지 판단한다.
            if bool(self.get_parameter('use_server_battery_policy').value):
                self.handover_pub.publish(Int32(data=0))
                self.get_logger().warn(
                    f'충전 복귀 도킹 완료(배터리 {self.current_battery:.0f}%) → 교대요청(wp0) 발행 '
                    f'+ IDLE 대기 (배터리가 실제로 오르면 CHARGE_OBS 가 CHARGING 으로 바꾼다)')
                self.charging_start = None
                self.change_state(PatrolState.IDLE)
                return
            # ---- 옛 동작(use_server_battery_policy=False): 로봇이 배터리로 자기를 잠금 ----
            # ★배터리 정책(7/7 확정): 즉시복귀(33%↓)/수동복귀로 돌아온 경우에도
            #   배터리가 교대 임계(40%) 이하면 2호기 투입을 서버에 통보하고 충전.
            hthr = self.get_parameter('handover_battery_threshold').value
            if self.current_battery <= hthr:
                self.get_logger().warn(
                    f'충전 복귀 도킹 배터리 {self.current_battery:.1f}% ≤ {hthr}% '
                    f'→ 2호기 교대요청(wp0) 발행')
                self.handover_pub.publish(Int32(data=0))
                self._handover_sent = True   # ★감사픽스: 충전완료 후 자동 재출격 금지용
            self.get_logger().info('충전 복귀 정밀도킹 완료 → CHARGING (충전 단자 연결 대기)')
            self.charging_start = None
            self.change_state(PatrolState.CHARGING)
            return
        threshold = self.get_parameter('battery_threshold').value
        # ★DRIFT_RESET: 도킹존(0,0) 실제 도착 = 한 바퀴 완료 → initialpose 리셋
        self.publish_initialpose_if_docked('랩도킹')   # ★픽스B: 시그니처 확인 후에만
        self.dock_arrival_count += 1
        # ★MAX_LAPS: 지정 바퀴수만큼 도킹 완료 → 도킹존에 정지(주차). undock 안 함(다음 바퀴 안 나감).
        #   충전단자 꽂혀있으면 하드웨어로 충전됨. 재시작은 PATROL_START(카운트 리셋).
        max_laps = self.get_parameter('max_laps').value
        if max_laps > 0 and self.dock_arrival_count >= max_laps:
            # ★SWAP(7/15 사용자 확정 "돌고 와서 도킹 끝나면 다른 한 대가 교대 출발"):
            #   랩+도킹 완료 = 무조건 교대 신호 발행(handover_request).
            # ★SERVER_BATTERY(7/17 사용자 확정): "우린 도착하면 무조건 하도버 날려. 배터리가 없어서
            #   못 나가는 건 서버가 판단할 일" → 배터리로 CHARGING 자물쇠를 걸지 않고 항상 IDLE 대기.
            #   충전은 단자로 하드웨어가 하고, CHARGING 은 '배터리가 실제로 오르는 중'이라는
            #   관측 라벨로만 쓴다(_charge_watch_tick). 출동 판단은 서버 몫.
            if self.get_parameter('use_lap_swap').value:
                self.handover_pub.publish(Int32(data=0))
                if bool(self.get_parameter('use_server_battery_policy').value):
                    self.get_logger().info(
                        f'★교대 신호 발행(랩+도킹 완료, 배터리 {self.current_battery:.0f}%) '
                        f'→ 도킹존 대기(IDLE). 배터리가 오르면 CHARGE_OBS 가 CHARGING 으로 바꾼다')
                else:
                    hthr = self.get_parameter('handover_battery_threshold').value
                    if self.current_battery <= hthr:
                        self.get_logger().warn(
                            f'★교대 신호 발행 + 배터리 {self.current_battery:.0f}% ≤ {hthr}% → CHARGING 대기')
                        self.charging_start = None
                        self.change_state(PatrolState.CHARGING)
                        return
                    self.get_logger().info('★교대 신호 발행(랩+도킹 완료) → 도킹존 대기(IDLE)')
            self.get_logger().info(
                f'{self.dock_arrival_count}바퀴 순찰+도킹 완료 (max_laps={max_laps}) → '
                f'도킹존 정지(IDLE). 재시작=PATROL_START.')
            self.change_state(PatrolState.IDLE)
            return
        # ★HANDOVER: 도킹존 도착마다 배터리 검사 → ≤임계면 2호기 교대요청 + robot1 충전
        #   (max_laps 미도달 도킹 = 다바퀴 운용 시 경유. max_laps=1 이면 위 분기에서 이미 return)
        # ★SERVER_BATTERY(7/17): 여기도 '로봇이 배터리로 CHARGING 잠그던' 경로 → 정책 적용.
        #   "차징은 무조건 3프로 이상 올랐을 때만"(CHARGE_OBS) 이므로 여기서 CHARGING 을 걸지 않는다.
        if bool(self.get_parameter('use_server_battery_policy').value):
            self.handover_pub.publish(Int32(data=0))
            self.get_logger().info(
                f'도킹존 도착(배터리 {self.current_battery:.0f}%) → 교대요청(wp0) 발행 + IDLE 대기. '
                f'CHARGING 은 배터리가 실제로 오를 때만(CHARGE_OBS)')
            self.charging_start = None
            self.change_state(PatrolState.IDLE)
            return
        hthr = self.get_parameter('handover_battery_threshold').value
        if self.current_battery <= hthr:
            resume_wp = 0   # robot2는 새 바퀴(wp0)부터 이어받음
            self.get_logger().warn(
                f'도킹 배터리 {self.current_battery:.1f}% ≤ {hthr}% → '
                f'2호기 교대요청(wp{resume_wp}) 발행 + CHARGING')
            self.handover_pub.publish(Int32(data=int(resume_wp)))
            self._handover_sent = True   # ★감사픽스: 충전완료 후 자동 재출격 금지용
            self.charging_start = None
            self.change_state(PatrolState.CHARGING)
            return
        # 다음 wp(원점 다음=wp0)로
        self.current_waypoint_index = \
            (self.current_waypoint_index + 1) % len(self.waypoints)
        if self.current_battery < threshold:   # ★BATTERY: 33% 미만이면 충전복귀
            self.get_logger().warn(f'배터리 {self.current_battery:.1f}% < {threshold}% → LOW_BATTERY')
            self.save_waypoint_index()
            self.change_state(PatrolState.LOW_BATTERY)
        elif self.dock_arrival_count >= 2:   # ★DWELL: 1바퀴 후(2회째부터) 대기
            self.get_logger().info(
                f'도킹존 {self.dock_arrival_count}회째 도착 → '
                f'{self.get_parameter("dock_dwell_sec").value}s 대기 후 순찰')
            self.dock_dwell_start = None
            self.change_state(PatrolState.DOCK_DWELL)
        else:
            # ★UNDOCK(P2): 도킹존(0,0)서 다음 바퀴 출발도 빨강칸 → executor면 전진 undock 먼저
            self.change_state(self._post_dock_state())

    # ================================================================
    # 메인 상태 루프
    # ================================================================
    # ================================================================
    # ★ROUTE_SERVO: AGV식 직진 주행부 (use_route_servo=True일 때 PATROLLING 주행 대체)
    #   원본: ~/team_ws/route_servo_lap.py (7/6 실주행 2랩 무결점 + 랩→도킹 풀사이클 검증).
    #   블로킹 루프를 20Hz 타이머 상태기계로 이식. FSM 전이는 기존 그대로 사용:
    #   - 랩 완료(마지막 노드=프리도킹 도착) → ARRIVED (is_predock 판정·도킹 게이트 기존 흐름)
    #   - 구간 실패(타임아웃/전방급정지) → 자체 2회 재시도 → RETRYING(→소진 시 ESCAPE)
    #   - PATROLLING 이탈(이벤트/장애물/일시정지) → 즉시 정지+랩 리셋, 복귀 시 최근접 노드부터
    #   감시견(_update_stuck_watchdog)은 goal_in_flight 조건이라 서보 중 자동 비활성 —
    #   서보 자체 타임아웃→RETRYING→ESCAPE 경로가 그 역할을 대신함.
    # ================================================================
    def _rs_scan_cb(self, m):
        vals = []
        rear = []
        ctr = []    # ★EVENT_FINALIZE(7/9): 전방 협각(±8°) — 이벤트 60cm 거리조정용(중앙값=노이즈 강건)
        # ★UNWEDGE(7/10): 30° × 12섹터 최소거리 — '어느 쪽이 넓은가'를 판단해 끼임 탈출 방향을 고른다.
        sect = [None] * RS_SECTORS
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < m.range_max):
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment) % 360.0
            k = int(a / (360.0 / RS_SECTORS)) % RS_SECTORS
            if sect[k] is None or r < sect[k]:
                sect[k] = r
            if a <= RS_FRONT_HALF_ANG or a >= 360.0 - RS_FRONT_HALF_ANG:
                vals.append(r)
                if a <= 8.0 or a >= 352.0:
                    ctr.append(r)
            elif 180.0 - RS_FRONT_HALF_ANG <= a <= 180.0 + RS_FRONT_HALF_ANG:
                rear.append(r)   # ★후방 섹터(±25°) — 이벤트 BACKOFF(후진 거리확보) 가드용
        self._rs_sect = sect
        self._rs_front = min(vals) if vals else None
        self._rs_rear = min(rear) if rear else None
        self._rs_front_ctr = sorted(ctr)[len(ctr) // 2] if ctr else None
        self._last_scan = m   # ★EVENT_FINALIZE: 벽피팅(평행트림)용 원본 보관
        self._scan_last_arrival = self._rs_now_sec()   # ★SCAN_WD(7/15): 수신 시각(도착 기준, 링크 실명 감시)

    def _rs_pose(self):
        # map→base_link TF에서 (x, y, yaw). 실패(예열 전/유실) 시 None.
        try:
            t = self._rs_tfb.lookup_transform('map', 'base_link', TfTime())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _rs_now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_safety_event(self, msg):
        """★SAFETY_EVENTS(7/13): 서버 1인칭 고신뢰 검출 수신 → bearing 주입.
        서버 필터("중앙 1/3과 60% 중첩 + 최소크기") 통과분만 오므로 사실상 '발견 확정' 신호.
        이벤트 접근/탐색 국면 + 현재 임무 타입 일치 시에만 수용(다른 객체에 낚임 방지).
        bearing 은 /event/bearing 과 동일 규약: (0.5 - cx/width)·hfov, 화면중앙 기준, 좌+."""
        if not self.get_parameter('use_server_safety_events').value:
            return
        if self.current_state != PatrolState.MOVING_TO_EVENT:
            return
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn('★safety_events JSON 파싱 실패', throttle_duration_sec=10.0)
            return
        # ★7/13 패치문서: event_type 이 소문자(no_helmet/fire/fall)로 변경 — 대소문자 무관 정규화.
        #   (구 문서의 HELMET 표기 대비 별칭도 흡수. 파견 서비스 실측 타입 = NO_HELMET/FALL/FIRE)
        et = (d.get('event_type') or '').strip().upper()
        if et in ('HELMET', 'HEAD'):
            et = 'NO_HELMET'
        if et != (self.dispatch_event_type or 'EVENT').strip().upper():
            return
        cp = d.get('center_px') or []
        w = float((d.get('image_size') or {}).get('width') or 0)
        if not cp or w <= 0:
            return
        hfov = math.radians(float(self.get_parameter('safety_events_hfov_deg').value))
        b = (0.5 - float(cp[0]) / w) * hfov
        self._push_bearing(b)   # ★BEARING_MED3: 단일 프레임 직주입 금지 — 중앙값 경유
        # ★SNAP_STOP(7/13, 사용자: "토픽 오면 멈춰 제발"): 서버 고신뢰 보고 = 크기 필터
        #   (no_helmet 105×100px / fire·fall 세로 240px)를 통과했다 = 대상이 프레임에 크게
        #   잡혀 있다 = **지금이 촬영각**. 접근 중이면 즉시 끊고 마무리(정렬→60cm→수평)로.
        # ★7/13 사용자: "이멀전시 스탑처럼 바로" — 선응시·Nav2 우회 접근 중이어도 무조건 발동.
        #   (구버전은 서보 DRIVE 중에만 발동해 반응이 국면에 따라 늦었다)
        if (self.get_parameter('use_safety_event_stop').value
                and not self._event_centering):
            # ★PHOTO_SPOT(7/14): 촬영지점(goal)까지 아직 멀면 신호정지 금지 — 큰 포스터는
            #   멀리서도 서버 크기필터를 통과해서(11:57 실측 1.5m 잔여에 스냅) 원거리 사진이 된다.
            #   bearing 은 이미 주입됐으니 시각추종으로 계속 접근, 정지는 goal 근처에서만.
            _rem = None
            _pp = self._rs_pose()
            if _pp is not None and self.dispatch_target is not None:
                _gp = self.dispatch_target.pose.position
                _rem = math.hypot(_gp.x - _pp[0], _gp.y - _pp[1])
            if (_rem is not None
                    and _rem > float(self.get_parameter('snap_stop_max_remaining').value)):
                self.get_logger().info(
                    f'★고신뢰 검출({et}) — 촬영지점까지 잔여 {_rem:.2f}m: 신호정지 보류, '
                    f'bearing 주입 후 계속 접근')
                return
            self.get_logger().warn(
                f'★고신뢰 검출 수신({et}, conf={float(d.get("confidence") or 0):.2f}, '
                f'방위 {math.degrees(b):+.1f}°) → 접근 즉시 중단(전 국면), 촬영 마무리 진입')
            self._snap_stop = True   # ★7/13 사용자: "60cm보다 앞선 정지" — 거리조정 전진 금지
            self.cancel_current_goal()      # Nav2 우회 중이었다면 goal도 즉시 끊기
            self._event_preface = False     # 선응시 중이었다면 그 국면 종료
            self._rs_stop()
            self._rs_evt_arrived()
            # ★STOP_THEN_FIND(7/13, 사용자: "멈추고 찾는 걸로"): 1초 완전 정지 후 정렬 시작
            #   — 정지 상태의 안정된 프레임으로 새 검출을 받아 찾기 정확도를 올린다.
            self._center_hold_until = self._rs_now_sec() + 1.0
            return
        self.get_logger().info(
            f'★고신뢰 검출 수신({et}, conf={float(d.get("confidence") or 0):.2f}) '
            f'→ 방위 {math.degrees(b):+.1f}° 주입(발견 확정)')

    def _charge_watch_tick(self):
        """★CHARGE_OBS(7/17, 사용자: "차징의 정의를 바꾸자 — 도킹존에서 배터리가 올라가면 차징으로"):
        CHARGING = 정책('배터리 낮으니 못 나감')이 아니라 **관측 사실**('지금 실제로 충전되는 중').
        도크존 안에서 배터리가 charge_progress_delta(1%) 이상 오르면 IDLE → CHARGING 으로 라벨링.
        (도킹 완료 직후엔 _on_dock_zone_arrived 가 이미 CHARGING 을 걸므로, 여기서 잡는 건
         '도크에 있는데 IDLE 인' 경우 — 예: 서버가 RESET 후 출동을 안 시켰거나 충전완료 후 재상승.)
        나가는 건 여전히 IDLE 에서만 — 출동시킬 땐 서버가 RESET 으로 IDLE 로 바꾼다."""
        if not bool(self.get_parameter('use_server_battery_policy').value):
            return
        # ★위치 게이트 제거(7/17 사용자 확정 "위치게이트 빼"):
        #   _at_dock_zone() 은 map pose 가 없으면 **odom 으로 폴백**하는데, odom 원점은 도크가 아니라
        #   '부팅한 자리'다 → 좌표계를 섞어 비교하게 된다(실측: 로봇1 odom (0.23,-0.20)=원점서 0.31m
        #   > 반경 0.15 → 도크에 꽂혀 충전 중인데도 CHARGE_OBS 가 통째로 스킵됐다. 로봇2는 odom 이
        #   우연히 0 근처라 통과 = 우연에 기댄 판정).
        #   물리적으로 로봇은 도크에서만 충전된다 → '배터리가 오른다' 자체가 '도크에 꽂혀 있다'의
        #   증거다. 위치는 정보를 더해주지 않고 AMCL/odom 이 어긋났을 때 오판만 만든다.
        b = float(self.current_battery)
        if self._chg_base is None or b < self._chg_base:
            self._chg_base = b         # 최저점을 기준선으로 — 여기서 오르면 '충전 중'
            return
        if self.current_state != PatrolState.IDLE:
            return
        delta = float(self.get_parameter('charge_obs_delta').value)
        if b >= self._chg_base + delta:
            self.get_logger().info(
                f'★CHARGE_OBS: 도크존에서 배터리 상승 감지 ({self._chg_base:.1f}% → {b:.1f}%, '
                f'+{delta}% 이상) → CHARGING 라벨. 출동은 서버가 IDLE 로 바꾼 뒤 PATROL_START')
            self._chg_base = b
            self.charging_start = None
            self.change_state(PatrolState.CHARGING)

    def _on_detections_slow(self, msg):
        """★DET_SLOW(7/17): 검출이 '보이는 동안'만 시각을 갱신 — 속도상한은 _rs_tick 이 건다.
        여기서 상태전이·파견은 절대 하지 않는다(그건 fusion 몫). 감속 판단용 시각 하나만 든다."""
        if not self.get_parameter('use_detect_slow').value:
            return
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # detections=[] 하트비트가 상시 오므로 '비어있지 않을 때'만 감속 시각 갱신
        if d.get('detections'):
            self._det_last_t = self._rs_now_sec()

    def _on_proximity(self, msg):
        """★PROXIMITY(7/13): 글로벌캠 근접경보. too_close(5Hz) → 즉시 정지(PAUSED|PROXIMITY),
        양보측(임무 없는 쪽, 동급이면 robot_id 큰 쪽)은 가드 통과 후진으로 벌어질 때까지 물러남.
        cleared(8연발) 또는 스트림 끊김(proximity_timeout) → 하던 일 재개(_estop_resume_state 재사용)."""
        # ★PROX_DIAG(7/17, 사용자 요청): 아래 return 들은 전부 '조용히 버리기'였다 —
        #   경보가 와도 로그가 0줄이라 "왜 안 멈췄나"를 매번 추측해야 했다(7/17 실측).
        #   판단 로직은 그대로 두고 '버린 이유'만 남긴다(throttle 로 도배 방지).
        #   ★수신 증명: 이 한 줄이 있으면 "서버가 안 쐈나 / 로봇이 버렸나"가 로그로 갈린다.
        self.get_logger().info(f'★근접경보 수신: {msg.data[:120]}', throttle_duration_sec=2.0)
        if not self.get_parameter('use_proximity_stop').value:
            self.get_logger().warn('★근접경보 무시: use_proximity_stop=False (노브 OFF)',
                                   throttle_duration_sec=5.0)
            return
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f'★근접경보 무시: JSON 파싱 실패 (raw={msg.data[:80]!r})',
                                   throttle_duration_sec=5.0)
            return
        st = (d.get('state') or '').strip().lower()
        if st == 'too_close':
            # ★PROX_DETOUR(7/15): 우회 전환 직후엔 경보 무시(핑퐁 방지) — 라이다 가드가 안전 담당
            if self._rs_now_sec() < self._prox_snooze_until:
                self.get_logger().warn('★근접경보 무시: 우회 스누즈 중', throttle_duration_sec=5.0)
                return
            self._prox_last_t = self._rs_now_sec()
            if self._prox_active:
                return          # 이미 정지 중 = 정상(재발행 수신). 로그 불필요.
            self._prox_start_t = self._rs_now_sec()
            # 주행 계열 상태에서만 개입 (도킹/충전/주차/탈출 중엔 불간섭)
            ACT = (PatrolState.PATROLLING, PatrolState.MOVING_TO_EVENT,
                   PatrolState.RESUMING, PatrolState.RETURNING_TO_CHARGER,
                   PatrolState.ARRIVED)
            if self.current_state not in ACT:
                self.get_logger().warn(
                    f'★근접경보 무시: 주행상태 아님({self.current_state.name}) — '
                    f'도킹/충전/대기 중엔 불간섭(설계)', throttle_duration_sec=5.0)
                return
            pair = d.get('turtlebot_pair') or []
            ids = [int(p.get('robot_id') or 0) for p in pair]
            other = next((i for i in ids if i != int(self.robot_id)), 0)
            busy = self.current_state == PatrolState.MOVING_TO_EVENT
            # 양보 규칙(회신 그대로): 임무 중이면 안 물러남(정지만), 아니면 robot_id 큰 쪽이 후진.
            self._prox_yield = (not busy) and (other != 0) and (int(self.robot_id) > other)
            self._prox_prev = self.current_state
            self._prox_active = True
            self.cancel_current_goal()
            self._rs_stop()
            self.get_logger().warn(
                f'★근접경보 too_close(거리 {float(d.get("distance") or 0):.2f}m, 상대 robot{other}) '
                f'→ 즉시 정지 ({"양보측: 후진" if self._prox_yield else "대기측: 정지 유지"})')
            self.change_state(PatrolState.PAUSED, reason='PROXIMITY')
        elif st == 'cleared' and self._prox_active:
            self.get_logger().info(
                f'★근접경보 해제(cleared, 거리 {float(d.get("distance") or 0):.2f}m) → 재개')
            self._prox_release()

    def _prox_release(self):
        """근접경보 해제 — 경보 직전 하던 일로 복귀(ESTOP 재개 매핑 재사용)."""
        prev = self._prox_prev
        self._prox_active = False
        self._prox_yield = False
        self._prox_prev = None
        if self.current_state == PatrolState.PAUSED and prev is not None:
            nxt = self._estop_resume_state(prev)
            self.get_logger().info(f'★근접경보 재개: {prev.name} → {nxt.name}')
            self.change_state(nxt)

    def _cam_gate_tick(self):
        """★CAM_GATE: Pi camera_gate.py 가 구독해 센더를 켜고 끈다.
        게이트 OFF(기본, 7/17 사용자 확정)=상태 무관 항상 ON 발행.
          ※ 발행을 '멈추면' 안 된다 — 이미 OFF 받고 대기중인 로봇이 꺼진 채 영영 안 켜짐.
        게이트 ON(use_cam_gate:=true)=7/15 동작: 대기(IDLE/CHARGING) OFF, 그 외 ON."""
        if not self.get_parameter('use_cam_gate').value:
            self.cam_enable_pub.publish(Bool(data=True))
            return
        off = self.current_state in (PatrolState.IDLE, PatrolState.CHARGING)
        self.cam_enable_pub.publish(Bool(data=not off))

    def _scan_watchdog_tick(self):
        """★SCAN_WD(7/15): scan 링크 실명 감시 — 주행 중 scan이 끊기면 즉시 정지(PAUSED/SCAN_LOST),
        안정 복귀하면 자동 재개. 링크 사망 시 '눈 감고 달리기'로 인한 벽박기/금지존 돌진 차단."""
        if not self.get_parameter('use_scan_watchdog').value:
            return
        now = self._rs_now_sec()
        stale = (self._scan_last_arrival is None
                 or now - self._scan_last_arrival > float(self.get_parameter('scan_stale_sec').value))
        if not self._scanwd_active:
            ACT = (PatrolState.PATROLLING, PatrolState.MOVING_TO_EVENT,
                   PatrolState.RESUMING, PatrolState.RETURNING_TO_CHARGER)
            # scan을 한 번도 못 받았으면(기동 직후) 발동 보류 — 최초 수신 후부터 감시
            if stale and self._scan_last_arrival is not None and self.current_state in ACT:
                self._scanwd_prev = self.current_state
                self._scanwd_active = True
                self._scanwd_ok_since = None
                self.cancel_current_goal()
                self._rs_stop()
                self.get_logger().error(
                    f'★SCAN_WD: scan {now - self._scan_last_arrival:.1f}s 무수신 — 링크 실명, '
                    f'즉시 정지(복귀 시 자동 재개)')
                self.change_state(PatrolState.PAUSED, reason='SCAN_LOST')
            return
        # ---- 실명 정지 중: 회복 감시 ----
        if self.current_state != PatrolState.PAUSED:
            # 외부 개입(ESTOP/RESET/관제)으로 상태가 바뀜 — 워치독 개입 종료
            self._scanwd_active = False
            self._scanwd_prev = None
            return
        if stale:
            self._scanwd_ok_since = None
            return
        if self._scanwd_ok_since is None:
            self._scanwd_ok_since = now
            return
        if now - self._scanwd_ok_since >= float(self.get_parameter('scan_recover_sec').value):
            prev = self._scanwd_prev
            self._scanwd_active = False
            self._scanwd_prev = None
            nxt = self._estop_resume_state(prev) if prev is not None else PatrolState.PATROLLING
            self.get_logger().warn(
                f'★SCAN_WD: scan 안정 복귀({self.get_parameter("scan_recover_sec").value:.0f}s 연속) '
                f'→ 자동 재개: {prev.name if prev else "?"} → {nxt.name}')
            self.change_state(nxt)

    def _prox_timeout_tick(self):
        """too_close 스트림이 끊기면(글로벌캠 다운 등) cleared 간주 — 영구 정지 방지 이중 방어."""
        if (self._prox_active and self._rs_now_sec() - self._prox_last_t
                > float(self.get_parameter('proximity_timeout').value)):
            self.get_logger().warn('★근접경보 스트림 끊김 — cleared 간주, 재개')
            self._prox_release()
            return
        # ★PROX_DETOUR(7/15, 서버문서 "제동 또는 우회" 채택): cleared가 hold_max 내 안 오면
        #   대기 해제 → 주행 재개(기존 라이다 회피: 감속0.40→정지→STUCK재시도→구간스킵이 우회 담당).
        #   양보측(후진중)은 제외 — 후진이 곧 cleared를 부른다.
        if (self._prox_active and not self._prox_yield
                and self.get_parameter('use_proximity_detour').value
                and self._rs_now_sec() - self._prox_start_t
                > float(self.get_parameter('proximity_hold_max').value)):
            snooze = float(self.get_parameter('proximity_snooze').value)
            self._prox_snooze_until = self._rs_now_sec() + snooze
            self.get_logger().warn(
                f'★근접경보 우회 전환: cleared {self.get_parameter("proximity_hold_max").value:.0f}s '
                f'미수신 → 대기 해제, 라이다 회피로 진행 (경보 무시 {snooze:.0f}s)')
            self._prox_release()

    def _push_bearing(self, b):
        """★BEARING_MED3(7/13, "불 가운데 안 맞음"): 큰 불 포스터는 bbox 중심이 프레임마다
        수백 px 요동(7/10 실측 228→576). 수신 방위각을 최근 3샘플 중앙값으로 스무딩해
        한 프레임 요동에 정렬이 낚이지 않게 한다(fusion 중앙값과 별개로 patrol 최종 방어)."""
        self._bearing_buf.append(float(b))
        if len(self._bearing_buf) > 3:
            self._bearing_buf.pop(0)
        self._event_bearing = sorted(self._bearing_buf)[len(self._bearing_buf) // 2]
        self._event_bearing_t = self._rs_now_sec()
        # ★AIMED_REACQ(7/14): 목격 순간의 map 절대방향 기억 — 유실 시 이 방향으로 조준 회전.
        #   (bearing 좌+, yaw CCW+ 같은 부호계라 단순 합. TF 예열 전이면 직전 값 유지)
        _p = self._rs_pose()
        if _p is not None:
            self._event_dir_map = self._norm(_p[2] + self._event_bearing)

    def _on_event_bearing(self, msg):
        """★VISUAL_CENTER: 융합노드 실시간 방위각[rad, 좌+] 수신 저장(시각 포함, staleness용)."""
        self._push_bearing(float(msg.data))   # ★BEARING_MED3

    def _fresh_event_bearing(self, now):
        """★VISUAL_CENTER: 최근 bearing_fresh_sec 내 수신한 방위각 반환, 유실이면 None.
        ★BEARING_FRESH(7/10, 실주행): 0.5s는 너무 짧았다. 대상이 화각 가장자리(±14.5°)에 있으면
        검출이 한두 프레임씩 빠지는데, 그때마다 '유실'로 보고 재탐색 스윕이 시작된다.
        스윕은 시작 yaw 기준 ±25°를 훑으므로 중앙정렬이 돌려놓은 각도를 되돌린다 — 둘이 싸운다.
        실증: NO_HELMET 방위각이 23초간 -13°에서 못 벗어나고 중앙정렬 타임아웃."""
        if (self._event_bearing is None
                or now - self._event_bearing_t
                > float(self.get_parameter('bearing_fresh_sec').value)):
            return None
        return self._event_bearing

    def _reacq_reset(self):
        self._reacq_cycle = 0
        self._reacq_accum = 0.0       # ★FULL_CIRCLE(7/13)
        self._reacq_last_yaw = None
        self._reacq_backup_p0 = None      # ★BACKUP_REACQ(7/13)
        self._reacq_backup_done = False
        self._reacq_aim_hold_t0 = None    # ★AIMED_REACQ(7/14)
        self._reacq_aim_t0 = None
        self._reacq_aim_done = False
        """스윕 상태만 지운다. '포기했다'(_reacq_gaveup)는 지우지 않는다 —
        지우면 다음 틱에 처음부터 다시 스윕해 무한루프가 된다(★REACQ_LOOPFIX)."""
        self._reacq_yaw0 = None
        self._reacq_dir = 1.0
        self._reacq_t0 = 0.0

    def _reacq_begin_episode(self):
        """새 중앙정렬 국면 시작 — 재탐색 기회를 다시 준다(CENTER/CENTER2 각각 1회씩)."""
        self._reacq_reset()
        self._reacq_gaveup = False
        self._reacq_success_t = 0.0   # ★TIMER_RESET(7/13): 이전 국면의 성공 시각 이월 방지
        self._fc_spin_until = 0.0     # ★FINE_CENTER(7/13): 펄스 정렬 상태 리셋
        self._fc_wait_until = 0.0
        self._cs_ok_since = 0.0       # ★STABLE_DONE(7/13)

    def _event_reacquire(self, now, p):
        """★REACQUIRE(7/10, 사용자 요구 "안 보이면 다시 찾아서 그 위치로 가야"):
        bearing 유실 시 저장된 map 좌표를 맹신하지 말고, 제자리에서 느리게 좌우로 훑어 대상을 재획득한다.
        (종전: 즉시 map-yaw 폴백 → 로봇이 '한 번 인식한 자리'만 계속 응시 = 잔상.)
        시작 yaw 기준 +max → -max 로 1왕복. 재획득하면 호출부가 정상 중앙정렬을 이어간다.
        반환 'turning'(탐색중) | 'fallback'(왕복·타임아웃 소진 — 옛 동작으로)."""
        if not self.get_parameter('use_event_reacquire').value or p is None:
            return 'fallback'
        amp = math.radians(float(self.get_parameter('reacquire_max_deg').value))
        w = float(self.get_parameter('reacquire_w').value)
        # ★FULL_CIRCLE(7/13, 사용자 확정 루틴): 창 왕복 스윕 대신 한 방향 360° 연속 회전.
        if bool(self.get_parameter('reacquire_full_circle').value):
            # ★PREFACE_SKIP(7/13 실주행 실측): 선응시(출발 전) 단계에서 한바퀴가 돌아
            #   출발도 못 하고 87.7s 소진. 파견 자리에선 대상이 안 보이는 게 정상(그래서
            #   파견된 것) — 한바퀴는 '도착 후' 전용. 선응시는 좌표 방향 개루프로 즉시 출발.
            if self._event_preface:
                self._reacq_gaveup = True
                return 'fallback'
            return self._reacq_full_circle(now, p, w)
        if self._reacq_yaw0 is None:
            self._reacq_yaw0 = p[2]
            self._reacq_dir = 1.0
            self._reacq_t0 = now
            self._reacq_cycle = 0
            self.get_logger().warn(
                f'★대상 유실 → 제자리 재탐색 시작(±{math.degrees(amp):.0f}°, {w:.2f}rad/s)')
        # ★REACQ_TIMEOUT_DERIVED(7/10): 타임아웃을 스윕 시간에서 유도한다.
        #   1왕복 = amp(+쪽) + 2·amp(-쪽) = 3·amp 라디안, 소요 = 3·amp/w.
        #   고정값으로 두면 w나 amp를 바꾸는 순간 '왕복을 끝낼 수 없는 타임아웃'이 되어 매번 포기한다
        #   (오전 실증: 왕복 8.7s vs 타임아웃 8.0s → 대상을 찾을 기회가 구조적으로 없었다).
        #   파라미터 값과 (유도값 ×1.3 + 2s) 중 큰 쪽 → 회전을 늦춰도 자동 추종.
        _sweep = 3.0 * amp / max(w, 1e-3)
        _tmo = max(float(self.get_parameter('reacquire_timeout').value), _sweep * 1.3 + 2.0)
        if now - self._reacq_t0 > _tmo:
            self.get_logger().warn(
                f'★재탐색 타임아웃({_tmo:.1f}s, 왕복 {_sweep:.1f}s) — 대상 못 찾음, 저장 좌표 방향으로 폴백')
            self._reacq_reset()
            self._reacq_gaveup = True   # ★REACQ_LOOPFIX: 이 정렬 국면 동안 재스윕 금지
            return 'fallback'
        dyaw = self._norm(p[2] - self._reacq_yaw0)
        if self._reacq_dir > 0 and dyaw >= amp:
            self._reacq_dir = -1.0            # 한쪽 끝 → 반대로 훑는다
        elif self._reacq_dir < 0 and dyaw <= -amp:
            # ★SEARCH_CYCLES(7/10, 사용자 요구): "무조건 멈추고 찾아야 돼. 없으면 다시 움직이고
            #   찾아서 가운데로 맞추고."  1왕복 실패 = 포기가 아니라 **탐색 창을 옮겨 다시 훑기**.
            #   전진 나들이는 금지존 쪽일 수 있어 위험(대상이 금지존 0.30m 앞인 사례 실측) →
            #   회전으로 창을 옮긴다. 창을 cycle 마다 (2·amp) 만큼 밀어 좌우로 넓게 훑는다.
            self._reacq_cycle += 1
            maxc = int(self.get_parameter('reacquire_cycles').value)
            if self._reacq_cycle < maxc:
                shift = 2.0 * amp
                self._reacq_yaw0 = self._norm(self._reacq_yaw0 - shift)
                self._reacq_dir = 1.0
                self._reacq_t0 = now
                self.get_logger().warn(
                    f'★재탐색 {self._reacq_cycle}/{maxc} 창 소진 — 창을 '
                    f'{math.degrees(shift):.0f}° 옮겨 다시 훑는다')
                self._rs_cmd(0.0, self._reacq_dir * w)
                return 'turning'
            self.get_logger().warn(
                f'★재탐색 {maxc}창 전부 소진 — 대상 못 찾음, 저장 좌표 방향으로 폴백')
            self._reacq_reset()
            self._reacq_gaveup = True   # ★REACQ_LOOPFIX: 이 정렬 국면 동안 재스윕 금지
            return 'fallback'
        self._rs_cmd(0.0, self._reacq_dir * w)
        return 'turning'

    def _reacq_full_circle(self, now, p, w):
        """★FULL_CIRCLE(7/13, 사용자: "제자리에서 한바퀴 돌면서 그 줬던 이벤트를 찾는걸로"):
        한 방향(CCW) 연속 회전으로 360° 를 훑는다. 검출되는 순간 호출부가 중앙정렬로 전환.
        누적 회전각은 틱마다 부호 있는 yaw 증분을 더해 wrap 에 안전하게 잰다
        (부호 합산이라 AMCL 지터는 상쇄된다). 타임아웃은 1바퀴 시간에서 유도 —
        reacquire_w 를 바꾸면 자동 추종(고정값이 스윕을 못 끝내던 7/10 전철 방지).
        반환 'turning'(회전중) | 'fallback'(한바퀴 소진 — 저장 좌표 방향으로)."""
        w = max(w, 1e-3)
        # ★BACKUP_REACQ(7/13): 유실 직후(방금까지 보였음)면 한바퀴 전에 온 방향 후진으로 재획득.
        #   후진도 _rs_cmd 가드(라이다∪코스트맵∪마스크)를 지나므로 뒤가 막히면 자동 0속도.
        if (bool(self.get_parameter('use_backup_reacquire').value)
                and not self._reacq_backup_done and self._reacq_yaw0 is None):
            _win = float(self.get_parameter('backup_reacq_window').value)
            _fresh_lost = (self._event_bearing_t > 0.0
                           and now - self._event_bearing_t < _win)
            if not _fresh_lost:
                self._reacq_backup_done = True   # 오래된 유실 — 바로 한바퀴로
            else:
                if self._reacq_backup_p0 is None:
                    self._reacq_backup_p0 = (p[0], p[1])
                    self.get_logger().warn(
                        f'★유실 직후 — 온 방향 후진 재획득(최대 '
                        f'{float(self.get_parameter("backup_reacq_dist").value):.2f}m)')
                _moved = math.hypot(p[0] - self._reacq_backup_p0[0],
                                    p[1] - self._reacq_backup_p0[1])
                if _moved >= float(self.get_parameter('backup_reacq_dist').value):
                    self.get_logger().warn('★후진 재획득 소진 — 한바퀴 탐색으로 전환')
                    self._reacq_backup_done = True
                    self._rs_cmd(0.0, 0.0, ramp=False)
                else:
                    self._rs_cmd(-0.05, 0.0)
                    return 'turning'
        # ★AIMED_REACQ(7/14, 사용자: "무조건 360이 아니라 보였던 곳으로, 회전각도 줄이면서"):
        #   한바퀴(79s) 전에 '마지막 목격 map 방향'으로 최단 조준 회전. 각속도는 잔여각에
        #   비례해 줄인다(0.18→0.06 하한) — 낡은 피드백(2Hz) 오버슛 방지. 도달하면 hold 동안
        #   완전 정지로 검출 대기(회전 중엔 스미어로 검출 안 되는 실측 반영). 재획득 시
        #   호출부(_visual_center_ctrl)가 중앙정렬로 전환, 실패 시에만 한바퀴(최후 폴백).
        _aim_dir = self._event_dir_map
        _aim_src = '마지막 목격'
        if _aim_dir is None and self._event_target_raw is not None:
            # ★AIMED_GLOBAL(7/14, 사용자: "글로벌캠 발견도 좌표 도착하면 이렇게 찾아줘"):
            #   로봇 카메라가 접근 중 한 번도 못 본(글로벌캠 파견) 이벤트는 목격 방향이 없다 —
            #   파견 좌표 방향으로 조준. 한바퀴는 그래도 없을 때만.
            _dx = self._event_target_raw[0] - p[0]
            _dy = self._event_target_raw[1] - p[1]
            if math.hypot(_dx, _dy) > 0.30:
                _aim_dir = math.atan2(_dy, _dx)
                _aim_src = '파견 좌표'
        if _aim_dir is None and self._event_face_yaw is not None:
            # ★PHOTO_SPOT(7/14): 좌표가 곧 촬영지점이라 그 위에 서 있으면(잔여<0.3m) 방향
            #   계산이 무의미 — 서버가 goal orientation 에 실어준 응시방향으로 조준.
            _aim_dir = self._event_face_yaw
            _aim_src = '파견 응시방향'
        if (bool(self.get_parameter('use_aimed_reacquire').value)
                and not self._reacq_aim_done and self._reacq_yaw0 is None
                and _aim_dir is not None):
            _hold = float(self.get_parameter('aimed_reacq_hold_sec').value)
            if self._reacq_aim_t0 is None:
                self._reacq_aim_t0 = now
                self.get_logger().warn(
                    f'★조준 재탐색: {_aim_src} 방향(map {math.degrees(_aim_dir):.0f}°)'
                    f'으로 최단 회전 — 한바퀴 생략 시도')
            err = self._norm(_aim_dir - p[2])
            if now - self._reacq_aim_t0 > 25.0 + _hold:
                # 안전망: 비례감속 최악(π 잔여)도 ~18s면 도달 — 여기 걸리면 yaw 이상
                self._reacq_aim_done = True
                self.get_logger().warn('★조준 재탐색 타임아웃 — 한바퀴 탐색으로 폴백')
            elif abs(err) > math.radians(3.0):
                self._reacq_aim_hold_t0 = None
                w_cmd = max(0.06, min(RS_TURN_MAX, 1.2 * abs(err)))
                self._rs_cmd(0.0, math.copysign(w_cmd, err))
                return 'turning'
            else:
                if self._reacq_aim_hold_t0 is None:
                    self._reacq_aim_hold_t0 = now
                self._rs_cmd(0.0, 0.0, ramp=False)
                if now - self._reacq_aim_hold_t0 < _hold:
                    return 'turning'
                self._reacq_aim_done = True
                self.get_logger().warn('★조준 방향에 대상 없음 — 한바퀴 탐색으로 폴백')
        if self._reacq_yaw0 is None:
            self._reacq_yaw0 = p[2]
            self._reacq_last_yaw = p[2]
            self._reacq_accum = 0.0
            self._reacq_t0 = now
            self.get_logger().warn(
                f'★대상 유실 → 제자리 한바퀴 탐색 시작({w:.2f}rad/s, 1바퀴 {2*math.pi/w:.0f}s)')
        if self._reacq_last_yaw is not None:
            self._reacq_accum += self._norm(p[2] - self._reacq_last_yaw)
        self._reacq_last_yaw = p[2]
        _tmo = (2.0 * math.pi / w) * 1.3 + 2.0
        if abs(self._reacq_accum) >= 2.0 * math.pi or now - self._reacq_t0 > _tmo:
            self.get_logger().warn(
                f'★한바퀴 탐색 소진(회전 {math.degrees(abs(self._reacq_accum)):.0f}°, '
                f'{now - self._reacq_t0:.1f}s) — 대상 못 찾음, 저장 좌표 방향으로 폴백')
            self._reacq_reset()
            self._reacq_gaveup = True   # 이 정렬 국면 동안 재스윕 금지(무한루프 방지)
            return 'fallback'
        self._rs_cmd(0.0, float(self.get_parameter('event_search_dir').value) * w)
        return 'turning'

    def _visual_center_ctrl(self, now, p, t0):
        """★VISUAL_CENTER: 카메라 실시간 방위각으로 중앙정렬 1스텝. _rs_cmd로 제자리 회전 발행.
        반환 'done'(정렬완료/타임아웃/발산) | 'turning'(회전중) | 'fallback'(bearing 유실+재탐색 실패)."""
        # ★STOP_THEN_FIND(7/13): 신호 정지 직후엔 잠깐 완전 정지 — 그 다음에 찾기 시작
        if self._center_hold_until > now:
            self._rs_cmd(0.0, 0.0, ramp=False)
            return 'turning'
        b = self._fresh_event_bearing(now)
        if b is None:
            # ★REACQ_LOOPFIX(7/10, 실주행 실증): 종전엔 여기서 곧장 재탐색으로 빠져
            #   ①아래 전체 타임아웃(t0)을 영원히 건너뛰고
            #   ②스윕 소진 후 _reacq_reset() 으로 상태가 지워져 다음 틱에 처음부터 다시 스윕
            #   → '유실→스윕→소진→폴백→유실→스윕' 무한루프(로그: 스윕 12회 / 소진 4회, 로봇 제자리 회전).
            #   전체 타임아웃은 재탐색 중에도 살아 있어야 하고, 한 번 포기하면 이 정렬 국면 동안
            #   다시 스윕하지 않는다(_reacq_gaveup).
            # ★CENTER_TIMEOUT_DERIVED(7/10): 전체 타임아웃도 '탐색 예산'에서 유도한다.
            #   창(cycles) × 1왕복(3·amp/w) × 1.2 + 여유. 고정 23.4s 면 창 3개를 다 못 돈다
            #   (오늘 네 번째로 '범위는 늘리고 타임아웃은 안 늘린' 실수를 할 뻔했다).
            _amp = math.radians(float(self.get_parameter('reacquire_max_deg').value))
            _w = max(float(self.get_parameter('reacquire_w').value), 1e-3)
            _cyc = int(self.get_parameter('reacquire_cycles').value)
            if bool(self.get_parameter('reacquire_full_circle').value):
                # ★FULL_CIRCLE(7/13): 예산도 한바퀴 시간에서 유도(창 스윕 공식은 무의미)
                _budget = (2.0 * math.pi / _w) * 1.3 + 6.0
            else:
                _budget = _cyc * (3.0 * _amp / _w) * 1.2 + 4.0
            if now - t0 > max(RS_TURN_TIMEOUT, _budget):
                self.get_logger().warn(
                    f'★중앙정렬 전체 타임아웃({max(RS_TURN_TIMEOUT, _budget):.1f}s, 재탐색 포함) '
                    f'— 현 방향으로 진행')
                self._reacq_reset()
                return 'fallback'
            if self._reacq_gaveup:
                return 'fallback'
            return self._event_reacquire(now, p)
        if self._reacq_yaw0 is not None:
            self.get_logger().info(
                f'★재탐색으로 대상 재획득(방위 {math.degrees(b):+.1f}°) — 중앙정렬 재개')
            self._reacq_reset()
            # ★TIMER_RESET(7/13 실주행): 한바퀴 탐색에 59s를 쓰면 아래 정렬 타임아웃(t0 기준)이
            #   이미 소진돼 '+6.3° 남기고 현 방향 도착' → 사진이 중앙에서 벗어남(실측).
            #   재획득 성공 시각을 기억해 정렬 시간을 새로 준다.
            self._reacq_success_t = now
        center_tol = math.radians(float(self.get_parameter('center_tol_deg').value))
        # ★AIM_OFFSET(7/13): 체계적 치우침 보정(기본 0)
        # ★AIM_OFFSET_PER_TYPE(7/14): 현재 임무 타입의 오버라이드가 있으면(≠999) 그걸 사용.
        _off = float(self.get_parameter('photo_aim_offset_deg').value)
        _t = (getattr(self, 'dispatch_event_type', '') or '').strip().lower()
        _key = {'fire': 'photo_aim_offset_fire', 'fall': 'photo_aim_offset_fall',
                'no_helmet': 'photo_aim_offset_no_helmet'}.get(_t)
        if _key is not None:
            _type_off = float(self.get_parameter(_key).value)
            if abs(_type_off) < 900.0:
                _off = _type_off
        b = b + math.radians(_off)
        if abs(b) <= center_tol:
            # ★STABLE_DONE(7/13): 순간 통과로 굳지 않는다 — center_stable_sec 동안
            #   정지 상태에서 새 검출까지 중앙을 유지해야 완료. 흐르면 아래 펄스로 재정렬.
            if self._cs_ok_since == 0.0:
                self._cs_ok_since = now
            if now - self._cs_ok_since < float(self.get_parameter('center_stable_sec').value):
                self._rs_cmd(0.0, 0.0, ramp=False)
                return 'turning'
            self.get_logger().info(
                f'★중앙정렬 완료(방위 {math.degrees(b):+.1f}°, '
                f'{now - self._cs_ok_since:.1f}s 유지 확인)')
            self._cs_ok_since = 0.0
            return 'done'
        self._cs_ok_since = 0.0
        # 발산 가드: |방위|가 관측 최소치보다 20°+ 커지면 부호 반대 의심 → 현 방향 도착(무한스핀 방지)
        if self._center_bmin is None or abs(b) < self._center_bmin:
            self._center_bmin = abs(b)
        if abs(b) > self._center_bmin + math.radians(20.0):
            self.get_logger().warn(
                f'★중앙정렬 발산({math.degrees(b):+.1f}°) — centering_sign 반대 의심, 현 방향 도착')
            return 'done'
        # ★TIMER_RESET(7/13): 재탐색(한바퀴)에 쓴 시간은 정렬 시간에서 제외 —
        #   재획득 성공 시각(_reacq_success_t) 이후로 RS_TURN_TIMEOUT 을 새로 잰다.
        if now - max(t0, self._reacq_success_t) > RS_TURN_TIMEOUT:
            self.get_logger().warn(f'★중앙정렬 타임아웃(잔여 {math.degrees(b):+.1f}°) — 현 방향 도착')
            return 'done'
        sign = float(self.get_parameter('centering_sign').value)
        # ★FINE_CENTER(7/13, 사용자: "박스 중앙을 화면 중앙에 더 맞춰"): 검출이 2Hz라
        #   미세 구간(±6°)에서 연속 회전하면 검출 사이 0.5s 동안 ≥3° 과회전 — 그래서
        #   허용오차를 못 줄였다. 필요한 각도만큼만 짧게 돌고(펄스) 멈춰서 다음 검출을
        #   기다리는 스텝 정렬로 ±1~2° 정밀도 확보.
        if abs(b) <= math.radians(6.0):
            if now < self._fc_spin_until:                 # 펄스 회전 중
                self._rs_cmd(0.0, self._fc_w)             # ★GENTLE_TURN: 램프 허용
                return 'turning'
            if now < self._fc_wait_until:                 # 정지 — 다음 검출 대기
                self._rs_cmd(0.0, 0.0, ramp=False)
                return 'turning'
            _pw = float(self.get_parameter('fine_pulse_w').value)
            # ★7/13 사용자: "회전 각도의 반으로" — 펄스당 잔여 오차의 절반만 회전(감쇠 수렴)
            dur = min(1.0, (abs(b) * 0.5) / max(_pw, 1e-3))
            self._fc_w = math.copysign(_pw, sign * b)
            self._fc_spin_until = now + dur
            self._fc_wait_until = now + dur + 0.8         # 회전 후 0.8s 정지(새 검출 수신)
            self._rs_cmd(0.0, self._fc_w)                 # ★GENTLE_TURN: 램프 허용(팍 안 돌게)
            return 'turning'
        _cmax = float(self.get_parameter('center_turn_max_w').value)   # ★GENTLE_TURN
        w = max(-_cmax, min(_cmax, RS_TURN_KP * sign * b))  # 좌+ 대상 → CCW(+)로 중앙
        _cmin = float(self.get_parameter('fine_pulse_w').value)
        if abs(w) < _cmin:
            w = math.copysign(_cmin, w)
        self._rs_cmd(0.0, w)
        return 'turning'

    def _preface_step(self):
        """★PREFACE(7/9): 파견 수락 직후 20Hz 1스텝 — 제자리에서 이벤트 정면 응시.
        bbox 폐루프(_visual_center_ctrl) 우선, bearing 유실 시 파견좌표 방향 개루프 폴백.
        완료 시 _event_preface 해제 → run_state_loop/_rs_tick이 접근을 시작한다."""
        now = self._rs_now_sec()
        p = self._rs_pose()
        if p is None:
            self._rs_cmd(0.0, 0.0)
            return
        st = self._visual_center_ctrl(now, p, self._event_preface_t0)
        if st == 'turning':
            return
        if st == 'fallback':
            # bearing 없음(검출 유실/프레임 사이) — 파견좌표 방향으로 개루프 조준
            t = self.dispatch_target
            if t is not None and now - self._event_preface_t0 <= RS_TURN_TIMEOUT:
                err = self._norm(math.atan2(t.pose.position.y - p[1],
                                            t.pose.position.x - p[0]) - p[2])
                if abs(err) >= RS_TURN_TOL:
                    w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
                    if abs(w) < RS_TURN_MIN:
                        w = math.copysign(RS_TURN_MIN, w)
                    self._rs_cmd(0.0, w)
                    return
            elif t is not None:
                self.get_logger().warn('★선응시 타임아웃 — 현 방향으로 접근 시작')
        self._rs_stop()
        self._event_preface = False
        # ★NEAR_SKIP(7/9): 목표가 이미 코앞(표준거리 도달 상태 파견)이면 주행 생략 —
        #   Nav2 goal이 현위치 수 cm 앞이라 소품 밀집 시 '무이동→ESCAPE 사방막힘→STUCK'
        #   (7/9 2차 FIRE 실증). 바로 도착 마무리(정렬→거리조정[후진 가능]→트림→촬영)로.
        t = self.dispatch_target
        p2 = self._rs_pose()
        if t is not None and p2 is not None:
            d = math.hypot(t.pose.position.x - p2[0], t.pose.position.y - p2[1])
            if d < self.get_parameter('patrol_xy_tolerance').value:
                q = t.pose.orientation
                self._event_target_yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.dispatch_target = None
                self.dispatch_retry_count = 0
                self._event_centering = True
                self._event_center_t0 = self._rs_now_sec()
                self._reacq_begin_episode()   # ★REACQ_LOOPFIX: 새 정렬 국면 → 재탐색 기회 1회 복원
                self._center_bmin = None
                self.get_logger().info(
                    f'★선응시 완료: 목표가 이미 {d:.2f}m 앞 — 접근 생략, 도착 마무리 시작')
                return
        self.get_logger().info('★선응시 완료(이벤트 정면) → 접근 시작')

    def _visual_center_step(self):
        """★NAV2_EVENT: Nav2 이벤트 도착 후 마무리 파이프라인 1틱(20Hz).
        ①중앙정렬(카메라 폐루프) → ②거리조정(라이다 실측 60cm) → ③평행트림(벽피팅 정면 대향) → PAUSED.
        ②③은 use_event_lidar_finalize 게이트(기본 ON). 각 단계 실패/유실 시 다음 단계로 안전 진행."""
        now = self._rs_now_sec()
        p = self._rs_pose()
        if p is None:
            self._rs_cmd(0.0, 0.0)
            return
        # ---- ② 거리조정 단계 ----
        if self._event_final_phase == 'RANGE':
            if self._event_range_step(now):
                # ★TRIM_UNDO(7/9): 트림 직전 yaw 기록 — 트림이 대상을 화각 밖으로 밀어내면
                #   CENTER2가 이 각도로 되돌아와 대상을 재획득한다(예전엔 그냥 포기하고 촬영).
                self._event_yaw_pre_trim = p[2]
                self._event_final_phase = 'TRIM'
                self._event_phase_t0 = now
            return
        # ---- ③ 평행트림 단계 ----
        if self._event_final_phase == 'TRIM':
            if not self.get_parameter('use_event_trim').value:
                # ★트림 제거: 곧장 최종 중앙정렬로. 대상을 정면으로 본 상태를 깨지 않는다.
                self.get_logger().info('★마무리: 평행트림 생략(대상 정면 유지) → 최종 중앙정렬')
                self._event_final_phase = 'CENTER2'
                self._event_center_t0 = now
                self._reacq_begin_episode()   # ★REACQ_LOOPFIX: 새 정렬 국면 → 재탐색 기회 1회 복원
                self._center_bmin = None
                return
            if self._event_wall_trim_step(now):
                # ★CENTER2(7/9): 거리조정·트림이 ①의 정렬을 틀어놓음(트림 최대 12° 회전)
                #   → 촬영 직전 최종 중앙정렬 1회 더 = '사진 정중앙' 보장
                self._event_final_phase = 'CENTER2'
                self._event_center_t0 = now
                self._reacq_begin_episode()   # ★REACQ_LOOPFIX: 새 정렬 국면 → 재탐색 기회 1회 복원
                self._center_bmin = None
                self.get_logger().info('★마무리: 평행트림 완료 → 최종 중앙정렬(촬영 직전)')
            return
        # ---- ④ 최종 중앙정렬(CENTER2) — 사진 정중앙 확보 후 PAUSED ----
        if self._event_final_phase == 'CENTER2':
            st = self._visual_center_ctrl(now, p, self._event_center_t0)
            if st == 'turning':
                return
            if st == 'fallback' and now - self._event_center_t0 <= 2.0:
                self._rs_cmd(0.0, 0.0)   # bearing 순간 유실 — 잠깐 제자리 대기(움직이지 않음)
                return
            if st == 'fallback' and self._event_yaw_pre_trim is not None:
                # ★TRIM_UNDO(7/9): 트림이 대상을 28° 화각 밖으로 밀어냈다 → 트림 직전 각도로 복귀.
                #   되돌아가는 도중 bearing이 돌아오면 위 _visual_center_ctrl 이 폐루프로 인계한다.
                err = self._norm(self._event_yaw_pre_trim - p[2])
                if abs(err) > math.radians(1.5) and now - self._event_center_t0 <= RS_TURN_TIMEOUT:
                    w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
                    if abs(w) < RS_TURN_MIN:
                        w = math.copysign(RS_TURN_MIN, w)
                    self._rs_cmd(0.0, w)
                    self.get_logger().info(
                        f'★최종 중앙정렬: bearing 유실 → 트림 되돌림({math.degrees(err):+.1f}° 남음)',
                        throttle_duration_sec=1.0)
                    return
                self.get_logger().warn('★최종 중앙정렬: 트림 되돌림 후에도 bearing 없음 — 현 방향 촬영')
            elif st == 'fallback':
                self.get_logger().warn('★최종 중앙정렬: bearing 유실 지속 — 현 방향 촬영')
            self._rs_cmd(0.0, 0.0)
            self._event_finalize_done()
            return
        # ---- ① 중앙정렬 단계 (기존) ----
        st = self._visual_center_ctrl(now, p, self._event_center_t0)
        if st == 'turning':
            return
        if st == 'fallback':
            # ★CENTER_FALLBACK(7/8): bearing 유실 시 '정지 대기'가 아니라 저장된 대상 방향(θ)으로
            #   개루프 회전해 물체 정면 응시(그래야 사진이 중앙에 잡힘). live bearing이 돌아오면
            #   _visual_center_ctrl이 먼저 잡아 폐루프로 인계(우선순위). 정렬완료/타임아웃 시 도착.
            if (self._event_target_yaw is not None
                    and now - self._event_center_t0 <= RS_TURN_TIMEOUT):
                err = self._norm(self._event_target_yaw - p[2])
                if abs(err) >= RS_TURN_TOL:
                    w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
                    if abs(w) < RS_TURN_MIN:
                        w = math.copysign(RS_TURN_MIN, w)
                    self._rs_cmd(0.0, w)
                    return
                self.get_logger().info('★중앙정렬: bearing 유실 → 대상방향(map-yaw) 정렬로 도착')
            else:
                self.get_logger().warn('★중앙정렬: 검출 유실 지속 — 현 방향 도착 처리')
        # 'done' 또는 fallback 타임아웃 → 중앙정렬 완료. 게이트 ON이면 ②로, OFF면 기존대로 즉시 도착.
        if self.get_parameter('use_event_lidar_finalize').value:
            self._rs_stop()
            self._event_final_phase = 'RANGE'
            self._event_phase_t0 = now
            self.get_logger().info('★마무리: 중앙정렬 완료 → 라이다 거리조정(60cm) 시작')
            return
        self._event_finalize_done()

    def _event_finalize_done(self):
        """★EVENT_FINALIZE: 마무리 파이프라인 종료 공통 처리 — 정지 + PAUSED(촬영 대기)."""
        self._rs_stop()
        self._event_centering = False
        self._event_final_phase = None
        self.active_type_pub.publish(String(data=''))   # ★MISSION_TYPE: 임무 종료 방송
        self.get_logger().info('파견 현장 도착+마무리 완료 -> PAUSED')
        self.change_state(PatrolState.PAUSED, reason=self._dispatch_reason())

    def _event_range_step(self, now):
        """★EVENT_FINALIZE ②: 전방 협각(±8°) 라이다 중앙값으로 대상과 60cm 실측 맞춤.
        카메라 목표좌표의 접근 누적오차(AMCL/근사K)를 실거리 폐루프로 흡수. 이동 중 bearing이
        살아있으면 약한 yaw 보정으로 중앙 유지. 완료/스킵 시 True 반환(→TRIM)."""
        # ★SNAP_STOP v2(7/13, "불이 프레임에 잘려 중심이 치우침"): 신호 정지 지점에서
        #   **전진은 금지**(앞선 정지 원칙)하되, 대상이 60cm보다 가까우면 **후진만 허용** —
        #   물러나면 대상 전체가 프레임에 들어와 bbox 중심이 안정된다.
        if getattr(self, '_snap_stop', False):
            _d = self._rs_front_ctr
            _tgt = float(self.get_parameter('event_stop_range').value)
            if _d is None or _d >= _tgt - float(self.get_parameter('event_range_tol').value):
                self.get_logger().info('★거리조정: 신호 정지 유지(전진 금지, 후진 불필요)')
                return True
            # 가깝다 → 아래 일반 로직의 후진 분기만 타도록 통과 (err<0 경로, 전진 err>0은 못 옴)
        tgt = float(self.get_parameter('event_stop_range').value)
        tol = float(self.get_parameter('event_range_tol').value)
        d = self._rs_front_ctr
        # ★RANGE_DUAL(7/13 실주행 실측): 누운 소품(FALL)은 라이다 평면(~15cm) **아래**라
        #   라이다가 소품 너머 벽까지 잰다 — 실측 0.551m '합격' 판정인데 map거리는 0.445m
        #   (= 60cm 미준수, 사용자 목격). 대상 원위치까지 map거리와 라이다 중 **가까운 쪽**을
        #   실거리로 채택 → 낮은 소품도 60cm 를 지킨다. 라이다 무응답(재질/각도)도 map거리로 대체.
        #   ★가드(7/13 벽박음 직후): map거리는 라이다 실측이 **있고 그보다 가까울 때만** 채택.
        #   라이다 무응답(사각 <0.25m/재질)일 때 map거리로 전진하면, 좌표가 유령(서버 점프)인
        #   경우 벽을 그대로 민다(실측 사고) — 그땐 옛 동작(스킵=정지)이 안전.
        pp = self._rs_pose()
        if d is not None and self._event_target_raw is not None and pp is not None:
            dm = math.hypot(self._event_target_raw[0] - pp[0],
                            self._event_target_raw[1] - pp[1])
            if dm < d:
                d = dm
        if now - self._event_phase_t0 > 12.0:
            self.get_logger().warn('★거리조정 타임아웃 — 현 위치로 진행')
            return True
        if d is None:
            # 협각에 라이다 반사 없음(대상 미검출/재질) — 1.5s 기다렸다 스킵
            if now - self._event_phase_t0 > 1.5:
                self.get_logger().warn('★거리조정: 전방 협각 라이다 무응답 — 스킵(카메라 위치 유지)')
                return True
            self._rs_cmd(0.0, 0.0)
            return False
        err = d - tgt   # +:멀다(전진), -:가깝다(후진)
        if abs(err) <= tol:
            self.get_logger().info(f'★거리조정 완료: 실측 {d:.3f}m (목표 {tgt:.2f}±{tol:.2f})')
            return True
        # ★KEEPOUT_SET(7/10): 전·후진 가드 모두 '라이다 ∪ 금지존 코스트맵' 세트로 판정.
        #   종전 후진 가드는 후방 라이다만 봐서 금지존을 그대로 밟았다(7/10 실주행 목격).
        if err > 0:
            why, blk = self._blocked_dir(
                +1.0, self._rs_front, max(RS_FRONT_STOP, tgt - tol), 0.30)
            if blk:
                self.get_logger().warn(f'★거리조정: 전방 막힘({why}) — 전진 중단, 현 거리 유지')
                return True
        if err < 0:
            why, blk = self._blocked_dir(-1.0, self._rs_rear, 0.22, 0.30)
            if blk:
                self.get_logger().warn(
                    f'★거리조정: 후방 막힘({why}) — 후진 중단, 현 거리 유지'
                    f'(실측 {d:.2f}m < 목표 {tgt:.2f}m, 더 못 물러남)')
                return True
        v = max(-0.06, min(0.06, 0.5 * err))
        # 이동 중 중앙 유지(bearing 살아있을 때만, 약한 게인)
        b = self._fresh_event_bearing(now)
        w = 0.0
        if b is not None:
            sign = float(self.get_parameter('centering_sign').value)
            w = max(-0.15, min(0.15, 0.8 * sign * b))
        self._rs_cmd(v, w)
        return False

    def _event_wall_trim_step(self, now):
        """★EVENT_FINALIZE ③: 전방(±35°) 라이다 점들 직선피팅 → 대상 표면 법선에 정면 대향
        (도킹 WALL_TRIM과 동일 원리 — '삐뚤게 서서 찍기' 방지). 로봇좌표 x=c+m·y 피팅,
        필요회전 = atan2(-m, 1). 상한(event_trim_max_deg) 초과면 스킵(중앙정렬 무효화 방지).
        완료/스킵 시 True 반환(→PAUSED)."""
        if now - self._event_phase_t0 > 8.0:
            self.get_logger().warn('★평행트림 타임아웃 — 현 방향으로 진행')
            return True
        m = self._last_scan
        d_ref = self._rs_front_ctr or 1.0
        if m is None:
            self._rs_cmd(0.0, 0.0)
            return False
        xs, ys = [], []
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < min(m.range_max, d_ref + 0.5)):
                continue   # 대상 평면 근방(전방 실측 +0.5m 이내)만 — 옆벽/배경 배제
            a = m.angle_min + i * m.angle_increment
            adeg = math.degrees(a) % 360.0
            if not (adeg <= 35.0 or adeg >= 325.0):
                continue
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))
        if len(xs) < 10:
            if now - self._event_phase_t0 > 2.0:
                self.get_logger().warn(f'★평행트림: 피팅점 부족({len(xs)}) — 스킵')
                return True
            self._rs_cmd(0.0, 0.0)
            return False
        my_ = sum(ys) / len(ys)
        mx_ = sum(xs) / len(xs)
        var_y = sum((y - my_) ** 2 for y in ys)
        if var_y < 1e-4:
            self.get_logger().warn('★평행트림: 가로 퍼짐 부족(점군 한 줄기) — 스킵')
            return True
        slope = sum((y - my_) * (x - mx_) for x, y in zip(xs, ys)) / var_y
        err = math.atan2(-slope, 1.0)   # 표면 법선 방향과 heading의 차
        if abs(err) <= math.radians(2.0):
            self.get_logger().info(f'★평행트림 완료: 잔여 {math.degrees(err):+.1f}°')
            return True
        if abs(err) > math.radians(float(self.get_parameter('event_trim_max_deg').value)):
            self.get_logger().warn(
                f'★평행트림: 필요회전 {math.degrees(err):+.1f}° > 상한 — 스킵(대상 프레임이탈 방지)')
            return True
        w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
        if abs(w) < RS_TURN_MIN:
            w = math.copysign(RS_TURN_MIN, w)
        self._rs_cmd(0.0, w)
        return False

    def _rs_cmd(self, v, w, ramp=True):
        # ★MASK_HARDGUARD(7/10): 금지존 마스크 = 절대 금지. 상태 무관, 면제 없음. 가장 먼저 건다.
        v = self._mask_guard(v)
        # ★MOVE_GUARD(7/10): 라이다∪코스트맵 검사(이벤트 범위). 마스크와 별개 층.
        v = self._move_guard(v)
        if ramp:
            # 가감속 램프: 계단명령이 바퀴슬립→odom yaw 오염→AMCL 붕괴로 이어지는 것 방지
            dv, dw = RS_ACC_V / RS_RATE, RS_ACC_W / RS_RATE
            v = max(self._rs_lv - dv, min(self._rs_lv + dv, float(v)))
            w = max(self._rs_lw - dw, min(self._rs_lw + dw, float(w)))
        self._rs_lv, self._rs_lw = float(v), float(w)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self._rs_cmd_pub.publish(m)

    def _estop_resume_state(self, prev):
        """★ESTOP_RESUME(7/10): 비상정지 직전 상태 → 재개할 상태.
        '하던 일 마저 한다'가 상태마다 다른 뜻이라 한 곳에 모아둔다.
          순찰/복구 계열  → RESUMING (서보는 _rs_full/_rs_leg 보존 → 끊긴 구간부터 이어달리기)
          이벤트 접근     → MOVING_TO_EVENT 재진입 (change_state 공통부가 PREFACE=선응시 재무장)
          도킹/언도킹     → 그대로 재진입 (ESTOP에서 dock_cmd_sent=False → 명령 재발행)
          그 외(대기 계열) → 그 상태 그대로
        복귀 대상이 불명확하면 순찰 재개가 가장 안전한 기본값."""
        S = PatrolState
        if prev is None:
            return S.RESUMING
        # 대상이 사라졌으면 이벤트 접근으로 못 돌아간다 → 순찰 재개
        if prev == S.MOVING_TO_EVENT:
            return S.MOVING_TO_EVENT if self.dispatch_target is not None else S.RESUMING
        if prev in (S.DOCKING, S.UNDOCKING, S.PAUSED, S.CHARGING, S.IDLE,
                    S.RETURNING_TO_CHARGER, S.LOW_BATTERY, S.DOCK_DWELL):
            return prev
        if prev == S.MANUAL_CONTROL:
            return S.PAUSED          # 수동 중 비상정지 → 수동 재진입은 관제가 명시적으로
        # PATROLLING / ARRIVED / RETRYING / STUCK / ESCAPE / OBSTACLE_WAITING / LOCALIZING …
        return S.RESUMING

    def _estop_hold_tick(self):
        """★ESTOP_FIX(7/10): 비상정지 동안만 100Hz로 0속도를 박는다.
        _rs_cmd를 거치지 않고 직접 발행 — 이동 가드/램프가 끼어들 여지를 없앤다(0은 언제나 안전)."""
        if self.current_state != PatrolState.EMERGENCY_STOP:
            return
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = 0.0
        m.twist.angular.z = 0.0
        self._rs_cmd_pub.publish(m)
        self._rs_lv = self._rs_lw = 0.0

    def _rs_stop(self):
        # 즉시 0속도(램프 없이) — 급정지/이탈 정리용
        self._rs_lv = self._rs_lw = 0.0
        self._rs_cmd(0.0, 0.0, ramp=False)

    def _rs_reset(self):
        self._rs_full = None
        self._rs_phase = None
        self._rs_leg = 0
        self._rs_seg_retries = 0
        self._rs_evt = False
        self._rs_lv = self._rs_lw = 0.0
        self._cd_block_since = None   # ★CLEAR_DETOUR: 서보 이탈 시 막힘 타이머도 리셋

    def _rs_bfs(self, a, b):
        """★GRAPH_ONLY(7/10): 엣지만 따라 a→b '최단거리' 노드열(양끝 포함). 경로 없으면 None.
        홉 수가 아니라 실제 주행거리로 가중(다익스트라) — 홉 최소는 먼 길을 고르고
        종점 노드를 지나쳤다 되돌아오는 이상한 경로를 만든다(예: [2,12] → 2,1,14,13,12)."""
        if a == b:
            return [a]
        adj = self._rs_adj or {}
        pts = self._rs_pts or {}

        def w(u, v):
            (x0, y0), (x1, y1) = pts[u], pts[v]
            return math.hypot(x1 - x0, y1 - y0)

        dist = {a: 0.0}
        prev = {a: None}
        pq = [(0.0, a)]
        seen = set()
        while pq:
            du, u = heapq.heappop(pq)
            if u in seen:
                continue
            seen.add(u)
            if u == b:
                path = [b]
                while prev[path[-1]] is not None:
                    path.append(prev[path[-1]])
                return path[::-1]
            for v in adj.get(u, ()):
                if v not in pts:
                    continue
                nd = du + w(u, v)
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return None

    def _rs_expand(self, nodes):
        """★GRAPH_ONLY(7/10): 노드열의 '엣지 아닌 인접쌍'을 엣지 경로로 펼친다.
        예: [2,12,13] → [2,3,...,12,13]. 펼칠 수 없으면(경로 없음) 그 쌍은 그대로 두고 경고
        (옛 동작=직선. 그래프가 끊긴 경우까지 순찰을 멈추진 않는다)."""
        adj = getattr(self, '_rs_adj', None) or {}
        if not adj or len(nodes) < 2:
            return nodes
        out = [nodes[0]]
        for b in nodes[1:]:
            a = out[-1]
            if b == a:
                continue
            if b in adj.get(a, ()):        # 이미 엣지 — 그대로
                out.append(b)
                continue
            # ★EXPAND_ONLY_IF_BLOCKED(7/10, 실주행): '엣지가 아니면 무조건 펼침'은 과했다.
            #   13→1 직선은 0.70m·금지존 0셀로 깨끗한데, BFS가 1.87m를 돌아 노드14(=충전소)를
            #   경유하게 만들어 **언도킹 직후 도킹존으로 되돌아갔다**(사용자: "돌면서 도킹존으로 들어가").
            #   원칙은 "그래프대로 가되 위험한 직선만 우회" — 직선이 금지존을 지날 때만 펼친다.
            pa, pb = self._rs_pts.get(a), self._rs_pts.get(b)
            # ★DIAG_BAN v2(7/13 실주행, 사용자: "노드 하나 건너뛰고 대각선으로 갔다·금지존 밟았다"):
            #   실측 2건 — 4→6(노드5 스킵), 1→12(맵 가로지름, 금지존 육안 침범).
            #   규칙: **엣지 경로가 있으면 무조건 엣지로 펼친다.** 직선 허용은 단 하나의 예외 —
            #   엣지 경로가 충전소 노드(route_avoid_via, 기본 14)를 관통할 때(13→1 = BFS가
            #   [13,14,1]로 도킹존을 경유하던 7/10 버그)뿐이며, 그때도 마스크 검사 통과 필수.
            path = self._rs_bfs(a, b)
            _avoid = {int(x) for x in
                      str(self.get_parameter('route_avoid_via').value).split(',')
                      if x.strip().lstrip('-').isdigit()}
            _via_avoid = (path is not None and any(n in _avoid for n in path[1:-1]))
            if (pa and pb and _via_avoid
                    and not self._mask_seg_blocked(pa[0], pa[1], pb[0], pb[1])):
                self.get_logger().info(
                    f'ROUTE_SERVO 노드{a}→{b}: 엣지 경로 {path} 가 충전소 경유 → '
                    f'직선 유지(마스크 통과 확인)')
                out.append(b)
                continue
            if path is None:
                self.get_logger().error(
                    f'ROUTE_SERVO 노드{a}→{b}: 엣지 경로 없음 — 직선 주행(금지존 위험)')
                out.append(b)
            else:
                self.get_logger().warn(
                    f'ROUTE_SERVO 노드{a}→{b}는 엣지가 아님 → 엣지 경로로 펼침 {path}')
                out.extend(path[1:])
        return out

    def _rs_build_lap(self):
        # 최근접 노드를 시작으로 route_nodes 순서의 랩 구성. TF 예열 전이면 False(다음 틱 재시도).
        if self._rs_pts is None:
            try:
                path = os.path.expanduser(self.get_parameter('route_graph').value)
                with open(path) as f:
                    g = json.load(f)
                self._rs_pts = {ft['properties']['id']: tuple(ft['geometry']['coordinates'])
                                for ft in g['features'] if ft['geometry']['type'] == 'Point'}
                # ★GRAPH_ONLY(7/10): 엣지 인접표. 서보는 '노드 점 사이 직선'만 그리므로,
                #   엣지가 아닌 점프(예: 재개 [2,12], 새 랩 [5,1])는 그래프에 없는 선이 되어
                #   금지존을 가로지른다(실측: 엣지 12구간=금지존 0셀, 비엣지 29쌍=금지존 통과).
                #   → 모든 점프를 엣지 경로(BFS)로 펼쳐서 '가란 대로'만 가게 한다.
                self._rs_adj = {}
                for ft in g['features']:
                    pr = ft.get('properties') or {}
                    if 'LineString' in ft['geometry']['type'] and 'startid' in pr:
                        self._rs_adj.setdefault(pr['startid'], set()).add(pr['endid'])
                self.get_logger().info(
                    f'ROUTE_SERVO 그래프 로드: 노드 {len(self._rs_pts)}개, '
                    f'엣지 {sum(len(v) for v in self._rs_adj.values())}개(방향)')
            except Exception as e:
                self.get_logger().error(f'ROUTE_SERVO 그래프 로드 실패({e}) → STUCK')
                self.change_state(PatrolState.STUCK)
                return False
        p = self._rs_pose()
        if p is None:
            return False   # AMCL localize/TF 예열 대기
        try:
            seq = [int(x) for x in str(self.get_parameter('route_nodes').value).split(',')]
        except ValueError:
            self.get_logger().error('ROUTE_SERVO route_nodes 파싱 실패 → STUCK')
            self.change_state(PatrolState.STUCK)
            return False
        # ★KEEPOUT_SERVO(7/9): '현재위치 → 최근접 노드' 진입 레그는 엣지가 아니라 생짜 직선이다.
        #   엣지는 금지존을 피해 찍혀 있지만 이 직선은 아무것도 안 봐서 금지존을 가로지른다
        #   (사용자: "노드-엣지 방식인데 왜 가란 대로 안 가"). → 가까운 순서로 훑어 '직선이 깨끗한'
        #   첫 노드를 시작점으로 삼는다. 전역 코스트맵 미수신이면 옛 동작(최근접) 유지.
        cands = sorted(self._rs_pts,
                       key=lambda i: (self._rs_pts[i][0] - p[0]) ** 2 + (self._rs_pts[i][1] - p[1]) ** 2)
        start = cands[0]
        if self._gcostmap is not None:
            for i in cands:
                if self._seg_blocked(p[0], p[1], self._rs_pts[i][0], self._rs_pts[i][1]) is False:
                    start = i
                    break
            else:
                self.get_logger().error(
                    'ROUTE_SERVO 진입: 금지존을 피하는 노드가 없음 — 최근접 사용(위험)')
            if start != cands[0]:
                self.get_logger().warn(
                    f'ROUTE_SERVO 진입: 최근접 노드{cands[0]} 직선이 금지존 통과 → 노드{start}로 우회 진입')
        else:
            self.get_logger().warn('ROUTE_SERVO 진입: 전역 코스트맵 미수신 — 금지존 검사 생략')
        # ★DOCK_ANCHOR(7/15 승인): 새 랩(fresh)이 도크 근처에서 시작하면(=언도크 직후) 시작
        #   앵커를 프리도킹 노드(seq[-1])로 고정. 노드12 이동 후 언도크 지점에서 12·13 최근접이
        #   박빙이라 랩이 [12,1,...]로 짜인 실주행 발생(7/15) — 도크 출발은 무조건 13부터.
        if self._rs_fresh and seq and start != seq[-1]:
            _ddk = math.hypot(p[0] - float(self.get_parameter('dock_x').value),
                              p[1] - float(self.get_parameter('dock_y').value))
            if _ddk < 0.6:
                self.get_logger().info(
                    f'★DOCK_ANCHOR: 도크 {_ddk:.2f}m 근처 새 랩 — 시작 앵커 노드{start}→{seq[-1]}(프리도킹) 고정')
                start = seq[-1]
        # ★도킹 재시도/RTC 정밀접근 맥락(_rs_dock_retry) — 랩 돌지 않고 프리도킹으로 직행.
        #   (조건 없이 "종점 근처=랩 생략"으로 하면 언도킹 직후(0.35,0)가 종점 13(0.55,0.006)에
        #    최근접이라 언도크→즉시 재도킹 무한루프가 됨 — 7/6 실주행에서 실제 발생, 플래그로 한정)
        #   ★RTC_PRECISE(7/9): start==seq[-1] 조건 제거 — Nav2 느슨 도착으로 최근접 노드가
        #   종점이 아니어도(못 미침) 직행 레그 [최근접→종점]으로 정밀 재접근.
        if self._rs_dock_retry:
            # ★자가교정(7/6밤): 실패 도킹은 로봇을 옆으로/비뚤게 남김 — 그 자리 재시도는 같은
            #   실패 반복(SRV_TURN 기하 엣지케이스=좌우 5cm+ 어긋남, 2연속 실증). 프리도킹
            #   노드로 서보 정밀 재접근(±7cm)해 어긋남을 스스로 교정한 뒤 도킹 재시도.
            gx, gy = self._rs_pts[seq[-1]]
            # ★KEEPOUT_SERVO(7/9): 직행 레그도 엣지가 아니다 — 금지존을 지나면 그래프로 되돌린다.
            # ★RTC_NEARDOCK(7/10): 단, 임계 90(=장애물 11cm 이내)은 충전기 주변 인플레이션에 항상 걸린다.
            #   도킹 직행은 원래 충전기로 다가가는 동작이라, '치명/금지존(99)'만 막아야 한다.
            #   그리고 로봇이 이미 빨강칸 안이면 자기 셀부터 막힘이므로 앞 0.15m는 건너뛴다.
            _blk = int(self.get_parameter('keepout_cost_block').value)
            _d = math.hypot(gx - p[0], gy - p[1])
            if _d > 0.16:
                _sx = p[0] + 0.15 * (gx - p[0]) / _d
                _sy = p[1] + 0.15 * (gy - p[1]) / _d
            else:
                _sx, _sy = p[0], p[1]
            blocked = self._seg_blocked(_sx, _sy, gx, gy, block=_blk) is True
            if blocked:
                # 직행 직선이 금지존을 지난다 → 직행 포기. 아래 일반 랩 구성(엣지 주행)으로 내려간다.
                self.get_logger().warn(
                    'ROUTE_SERVO 도킹 직행 레그가 금지존 통과 → 직행 취소, 그래프 엣지로 진입')
                self._rs_dock_retry = False
            elif math.hypot(gx - p[0], gy - p[1]) > RS_XY_TOL:
                self._rs_full = [start, seq[-1]]
                self._rs_leg = 0
                self._rs_seg_retries = 0
                self._rs_fresh = False
                self._rs_enter_turn()
                self.get_logger().info(
                    f'ROUTE_SERVO 도킹 재시도: 프리도킹 지점 정밀 재접근으로 자가교정 후 도킹')
                return True
            elif not blocked:
                self.get_logger().info(
                    f'ROUTE_SERVO 도킹 재시도: 시작위치=랩 종점(노드{start}) — 랩 생략, 즉시 도킹 판정')
                self._rs_lap_complete()
                return True
        # ★이어달리기(7/6): 중단 재개(_rs_fresh=False: 이벤트 복귀/구간 실패/ESCAPE 뒤)면
        #   최근접 노드의 다음부터 잔여 구간만 주행(전에는 매번 노드1부터 전체 랩 = 헛바퀴).
        #   새 랩(_rs_fresh=True: 언도킹/주차 후/랩 완료 후)은 기존대로 전체 랩.
        # ★RTC_NO_RESUME(7/13 실주행 버그): 충전 복귀 중엔 '이벤트 재개 노드' 무효 —
        #   배터리 31%에 노드1부터 풀랩(12구간)을 돌아 도크로 가는 최장 경로를 탔다(실측).
        #   RTC 는 최근접에서 도킹 방향 최단으로만 간다(아래 elif 경로).
        if self._rtc_docking:
            self._rs_resume_node = None
        if (not self._rs_fresh and self._rs_resume_node is not None
                and self._rs_resume_node in seq):
            # ★RESUME_SAVED(7/8): 이벤트 후 재개 — 최근접(이벤트 위치)이 아니라 '발견 당시 가던 노드'부터.
            #   현 위치(start=최근접)에서 저장노드로 이동 후 그 노드부터 순찰 순서대로 남은 구간 마저.
            rest = seq[seq.index(self._rs_resume_node):]       # 저장노드 포함 이후 전부
            full = [start] + [n for n in rest if n != start]
            self.get_logger().info(
                f'ROUTE_SERVO 이벤트 후 재개: 발견 당시 노드{self._rs_resume_node}부터 '
                f'잔여 {len(rest)}구간 (최근접 아님)')
            self._rs_resume_node = None
        elif not self._rs_fresh and start in seq and start != seq[-1]:
            rest = seq[seq.index(start) + 1:]
            full = [start] + rest
            self.get_logger().info(f'ROUTE_SERVO 재개: 노드{start}부터 잔여 {len(rest)}구간 이어달리기')
        else:
            full = [start]
            for nid in seq:
                if nid != full[-1]:
                    full.append(nid)
        self._rs_fresh = False
        # ★GRAPH_ONLY(7/10): 여기까지의 full은 '최근접 노드 + 순찰 순서'라 첫 점프가 엣지가
        #   아닐 수 있다(재개 [2,12] / 새 랩 [5,1]). 엣지 경로로 펼쳐 그래프 위로만 달린다.
        full = self._rs_expand(full)
        if len(full) < 2:
            self.get_logger().warn('ROUTE_SERVO 랩 노드열이 비어있음 — 즉시 랩완료 처리')
            self._rs_lap_complete()
            return True
        self._rs_full = full
        self._rs_leg = 0
        self._rs_seg_retries = 0
        self._rs_enter_turn()
        self.get_logger().info(f'★ROUTE_SERVO 랩 시작: {full} (총 {len(full) - 1} 구간)')
        return True

    def _rs_target(self):
        return self._rs_pts[self._rs_full[self._rs_leg + 1]]

    def _rs_label(self):
        if self._rs_evt:
            return '[이벤트접근]'
        return (f'[{self._rs_leg + 1}/{len(self._rs_full) - 1}] '
                f'{self._rs_full[self._rs_leg]}→{self._rs_full[self._rs_leg + 1]}')

    def _rs_enter_turn(self):
        self._rs_phase = 'TURN'
        self._rs_phase_t0 = self._rs_now_sec()

    def _rs_enter_drive(self, dist):
        self._rs_phase = 'DRIVE'
        self._rs_phase_t0 = self._rs_now_sec()
        self._rs_drive_timeout = dist / RS_DRIVE_V * 3.0 + 5.0
        self._cd_block_since = None   # ★CLEAR_DETOUR: 새 DRIVE 진입 — 막힘 타이머 리셋

    def _rs_enter_settle(self, next_phase):
        # 정지 틱(0.4s) — 멈춘 스캔으로 AMCL 보정 틈. 원본 stop(SETTLE_TICKS)와 동일 역할.
        self._rs_phase = 'SETTLE'
        self._rs_settle = RS_SETTLE_TICKS
        self._rs_next_phase = next_phase

    def _rs_seg_fail(self, why):
        self._rs_stop()
        if self._rs_evt:
            # ★이벤트 접근 실패 → 기존 파견 재시도 규약(_handle_dispatch_failure)에 위임.
            #   재시도면 상태가 MOVING_TO_EVENT로 남아 다음 틱에 접근을 처음부터 재시작.
            self.get_logger().warn(f'ROUTE_SERVO 이벤트 접근 실패({why})')
            self._rs_reset()
            self._handle_dispatch_failure()
            return
        self._rs_seg_retries += 1
        if self._rs_seg_retries <= 2:
            self.get_logger().warn(
                f'ROUTE_SERVO {self._rs_label()} 실패({why}) — 재시도 {self._rs_seg_retries}/2')
            self._rs_enter_turn()
        else:
            self.get_logger().error(
                f'ROUTE_SERVO {self._rs_label()} 실패({why}) — 재시도 소진 → RETRYING')
            self._rs_reset()   # RETRYING→PATROLLING 복귀 시 최근접 노드부터 재구성
            self.change_state(PatrolState.RETRYING)

    def _rs_lap_complete(self):
        # 랩 완료 = 마지막 노드(=프리도킹 wp 자리) 도착 → 기존 ARRIVED(is_predock) 흐름에 합류.
        self.get_logger().info('★ROUTE_SERVO 랩 완료 → ARRIVED(프리도킹 판정)')
        self._rs_dock_retry = False   # 재시도 티켓 소진(1회용)
        self._rs_fresh = True         # 다음 랩은 처음부터 전체 노드
        self._rs_reset()
        self.current_waypoint_index = len(self.waypoints) - 1
        self.change_state(PatrolState.ARRIVED)

    def _on_gcostmap(self, msg):
        self._gcostmap = msg

    def _g_cost_at(self, mx, my):
        """(map 프레임) 전역 코스트맵 cost. 맵 밖/미수신이면 None."""
        cm = self._gcostmap
        if cm is None:
            return None
        info = cm.info
        cx = int((mx - info.origin.position.x) / info.resolution)
        cy = int((my - info.origin.position.y) / info.resolution)
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return cm.data[cy * info.width + cx]

    def _seg_blocked(self, x0, y0, x1, y1, step=0.05, block=None):
        """★KEEPOUT_SERVO(7/9): map 프레임 직선 (x0,y0)->(x1,y1) 이 금지존/장애물을 지나는가.
        서보는 Nav2를 안 거쳐 코스트맵을 못 보므로, '엣지가 아닌 직선 레그'(현재위치→최근접노드,
        도킹 재시도 직행)가 금지존을 가로지를 수 있다. 그래프 엣지는 금지존을 피해 찍혀 있지만
        이 직선들은 아무것도 안 본다 — 실주행 금지존 침범의 원인.
        전역 코스트맵 미수신이면 None(판단 불가)."""
        if self._gcostmap is None:
            return None
        if block is None:
            block = self.get_parameter('escape_cost_block').value
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-6:
            return False
        n = max(1, int(d / step))
        for i in range(n + 1):
            t = i / n
            c = self._g_cost_at(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            if c is not None and c >= block:
                return True
        return False

    def _keepout_bearing(self, off, dist=0.35):
        """★KEEPOUT_SET(7/10): 로봇 기준 상대각 off[rad] 방향으로 dist[m] 선분이 금지존/치명을 지나는가.
        로봇 자기 셀(0~0.10m)은 건너뛴다 — 금지존 안에 서 있으면 모든 방향이 막힘으로 나와 못 나간다."""
        p = self._rs_pose()
        if p is None:
            return None
        blk = int(self.get_parameter('keepout_cost_block').value)
        th = p[2] + off
        x0 = p[0] + 0.10 * math.cos(th)
        y0 = p[1] + 0.10 * math.sin(th)
        x1 = p[0] + dist * math.cos(th)
        y1 = p[1] + dist * math.sin(th)
        return self._seg_blocked(x0, y0, x1, y1, block=blk)

    def _keepout_dir(self, sign=1.0, dist=0.35):
        """★KEEPOUT_SET(7/10, 사용자 원칙): '장애물'은 라이다 ∪ 금지존 코스트맵 세트로 본다.
        라이다는 금지존을 못 본다 — 물리 물체가 아니라 지도 위의 규칙이기 때문.
        그래서 라이다 가드만 둔 후진(BACKOFF/거리조정)이 금지존을 그대로 밟았다(7/10 실주행).
        진행방향(sign: +1 전진 / -1 후진) dist[m] 선분이 금지존/막힘을 지나면 True.
        ★임계는 keepout_cost_block(기본 99=치명/내접)을 쓴다 — escape_cost_block(90)은
          '장애물 11cm 이내'라 순찰선 코너에서 인플레이션 오탐이 난다.
        전역 코스트맵 미수신이면 None(판단 불가 — 호출부가 라이다만으로 판단)."""
        p = self._rs_pose()
        if p is None:
            return None
        return self._keepout_bearing(0.0 if sign > 0 else math.pi, dist)

    def _pose_in_keepout(self):
        """★KEEPOUT_SET(7/10): 로봇이 이미 금지존/치명 셀 위에 서 있는가.
        참이면 '움직이면 안 된다'가 아니라 '나가야 한다' → 이동 가드를 통과시킨다."""
        p = self._rs_pose()
        if p is None or self._gcostmap is None:
            return False
        c = self._g_cost_at(p[0], p[1])
        return c is not None and c >= int(self.get_parameter('keepout_cost_block').value)

    def _unwedge_pick(self):
        """★UNWEDGE(7/10, 사용자 요구): 12섹터를 라이다 ∪ 금지존으로 각각 판정해
        '안 막힌 곳 중 가장 넓은' 방향(상대각 rad)을 고른다. 없으면 None.
        서보는 경로계획을 못 하지만, 방향 하나 고르는 데는 계획이 필요 없다 —
        그리고 지금 끼어서 못 움직이는 당사자가 바로 Nav2다."""
        if not self._rs_sect:
            return None
        clear = float(self.get_parameter('unwedge_clear_min').value)
        look = float(self.get_parameter('unwedge_look').value)
        best_off, best_d = None, -1.0
        for k in range(RS_SECTORS):
            off = self._norm(k * (2.0 * math.pi / RS_SECTORS))
            d = self._rs_sect[k]
            d = 3.0 if d is None else d          # 반사 없음 = 열린 방향
            if d < clear:
                continue                          # 라이다로 막힘
            if self._keepout_bearing(off, look) is True:
                continue                          # 금지존/치명 — 라이다엔 안 보이는 막힘
            if d > best_d:
                best_off, best_d = off, d
        if best_off is not None:
            self.get_logger().info(
                f'★탈출 방향 선택: 상대 {math.degrees(best_off):+.0f}° (여유 {best_d:.2f}m)',
                throttle_duration_sec=1.0)
        return best_off

    def _servo_unwedge_tick(self, now):
        """★UNWEDGE(7/10): 서보로 조금씩 빼낸다. 넓은 쪽으로 제자리회전 → 5cm씩 전진 → 재평가.
        탈출되면 escape_return_state로 복귀. 실패하면 옛 Nav2 ESCAPE로 넘긴다.
        True 반환 = 이 틱을 여기서 처리함(호출부는 return)."""
        # ★ESTOP_LATCH_FIX(7/13 실주행): 비상정지 중에도 탈출 루프가 계속 돌다가 '성공 → RTC 복귀'로
        #   래치를 덮어썼다(ESTOP 9초 뒤 스스로 주행 재개, 실측). ESTOP 이면 즉시 침묵·정지.
        if self.current_state == PatrolState.EMERGENCY_STOP:
            self._uw_t0 = 0.0
            self._uw_p0 = None
            self._rs_stop()
            return True
        if not self.get_parameter('use_servo_unwedge').value or self._uw_gaveup:
            return False
        p = self._rs_pose()
        if self._uw_t0 == 0.0:
            self._uw_t0 = now
            self._uw_p0 = p            # ★출발 위치 기록 — '실제로 움직였는가' 판정 기준
            self.get_logger().warn('★서보 탈출 시작 — 막힌 방향 판단 후 넓은 쪽으로 조금씩 이동')
        # ★UNWEDGE_FIX(7/10): 제자리 회전만으로 _is_wedged()가 거짓이 되어 '성공'을 선언하던 버그.
        #   그러면 로봇은 한 발짝도 안 옮겼는데 Nav2로 돌아가 같은 자리서 또 막힌다
        #   → 무이동 7s → ESCAPE → 회전 → '성공' 무한루프(실주행 실증).
        #   기존 Nav2 ESCAPE도 '최소 1회 이동 후에만 판정'한다(_run_escape 주석) — 같은 규칙을 적용.
        moved = 0.0
        if p is not None and self._uw_p0 is not None:
            moved = math.hypot(p[0] - self._uw_p0[0], p[1] - self._uw_p0[1])
        min_move = float(self.get_parameter('unwedge_min_move').value)
        if moved >= min_move and not self._is_wedged():
            rs = self.escape_return_state
            self.get_logger().info(f'★서보 탈출 성공(이동 {moved:.2f}m) → {rs.name} 복귀')
            self._uw_t0 = 0.0
            self._uw_p0 = None
            self._rs_stop()
            self._rs_reset()          # 복귀 후 최근접 노드부터 재구성
            self.change_state(rs)
            return True
        # ★UW_TIMEOUT_DERIVED(7/10, 실주행): 고정 15s는 큰 회전각에서 부족했다.
        #   실측: -150° 회전에 13.1s(0.20rad/s) + 전진 0.15m/0.05 = 3.0s = 16.1s > 15s → 2cm만 움직이고 포기.
        #   최악(180°) 회전 + 최소이동 + 여유로 유도. 속도를 바꿔도 자동 추종(오늘 세 번째 같은 실수 방지).
        _uw_w = float(self.get_parameter('unwedge_w').value)
        _uw_v = float(self.get_parameter('unwedge_v').value)
        _need = math.pi / max(_uw_w, 1e-3) + min_move / max(_uw_v, 1e-3) + 3.0
        _tmo = max(float(self.get_parameter('unwedge_timeout').value), _need)
        if now - self._uw_t0 > _tmo:
            self.get_logger().error(
                f'★서보 탈출 타임아웃({_tmo:.1f}s, 이동 {moved:.2f}m) → Nav2 ESCAPE로 인계')
            self._uw_t0 = 0.0
            self._uw_p0 = None
            self._uw_gaveup = True    # 이 ESCAPE 국면 동안은 옛 경로가 담당
            return False              # 옛 경로(Nav2 Spin/DriveOnHeading)로 폴백
        off = self._unwedge_pick()
        if off is None:
            self.get_logger().error('★서보 탈출: 뚫린 방향 없음 — 정지 유지(관제 확인 필요)',
                                    throttle_duration_sec=3.0)   # ★7/13: 20Hz 로그 폭주 스로틀
            self._rs_stop()
            return True
        p = self._rs_pose()
        if p is None:
            self._rs_stop()
            return True
        if abs(off) > math.radians(12.0):
            self._rs_cmd(0.0, math.copysign(
                float(self.get_parameter('unwedge_w').value), off), ramp=False)
        else:
            self._rs_cmd(float(self.get_parameter('unwedge_v').value), 0.0, ramp=False)
        return True

    def _is_wedged(self):
        """★UNWEDGE(7/10): 지금 자리가 Nav2에 넘기기 위험한가 = 라이다 ∪ 코스트맵 세트 판정.
        ①금지존/치명 셀 위 ②전방 라이다 급정지권 ③전방 국소 코스트맵이 막힘.
        참이면 Nav2 goal 대신 ESCAPE로 먼저 빼낸다(무이동→ESCAPE 강제발동 예방)."""
        if self._pose_in_keepout():
            return True
        if self._rs_front is not None and self._rs_front < RS_FRONT_STOP:
            return True
        c = self._rs_cost_ahead()
        if c is not None and c >= int(self.get_parameter('escape_cost_block').value):
            return True
        return False

    # ==================================================================
    # ★MASK_HARDGUARD(7/10, 사용자 절대원칙: "어떤 일이 있어도 금지존을 밟으면 안 된다")
    # ------------------------------------------------------------------
    # 왜 코스트맵이 아니라 마스크인가:
    #   코스트맵은 금지존·벽 인플레이션·충전기 구조물을 한 덩어리(치명 셀)로 뭉갠다. 그래서
    #   코스트맵 가드를 켜면 도킹(빨강칸 진입)이 막히고, 그걸 피하려 면제를 뚫으면 그 면제가
    #   그대로 금지존 구멍이 된다(7/10 실증).
    #   금지존 마스크는 '지도 위의 규칙'이라 도킹존·순찰선과 겹치지 않는다(실측 확인:
    #   충전소(0,0)·노드13·노드14·언도크지점 모두 마스크 밖, 랩 엣지 12구간 마스크 통과 0).
    #   → 마스크만 보는 가드는 **면제가 필요 없다.** 도킹이든 랩이든 이벤트든 전부 막는다.
    # 유일한 예외: 이미 마스크 안에 서 있을 때(빠져나와야 하므로).
    # ==================================================================
    def _load_keepout_mask(self):
        """금지존 마스크(pgm+yaml)를 직접 적재. 코스트맵과 독립 — 코스트맵이 죽어도 살아있다."""
        try:
            import yaml as _yaml
            path = os.path.expanduser(self.get_parameter('keepout_mask_yaml').value)
            meta = _yaml.safe_load(open(path))
            img = meta['image']
            if not os.path.isabs(img):
                img = os.path.join(os.path.dirname(path), img)
            f = open(img, 'rb')
            assert f.readline().strip() == b'P5'
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            w, h = map(int, line.split())
            maxv = int(f.readline())
            data = f.read()
            self._km = dict(w=w, h=h, maxv=maxv, data=data,
                            res=float(meta['resolution']),
                            org=[float(v) for v in meta['origin']],
                            negate=int(meta.get('negate', 0)),
                            occ=float(meta.get('occupied_thresh', 0.65)))
            self.get_logger().info(
                f'★금지존 마스크 적재: {w}x{h} res={self._km["res"]} — 모든 이동에 하드가드 적용')
        except Exception as e:
            self._km = None
            self.get_logger().error(
                f'★금지존 마스크 적재 실패({e}) — 하드가드 비활성. 금지존 침범 위험!')

    def _mask_at(self, x, y):
        km = self._km
        if km is None:
            return False
        cx = int((x - km['org'][0]) / km['res'])
        cy = int((y - km['org'][1]) / km['res'])
        if not (0 <= cx < km['w'] and 0 <= cy < km['h']):
            return False
        px = km['data'][(km['h'] - 1 - cy) * km['w'] + cx]
        occ = px / km['maxv'] if km['negate'] else (km['maxv'] - px) / km['maxv']
        return occ >= km['occ']

    def _mask_near(self, x, y, clearance):
        """(x,y) 가 금지존에서 clearance[m] 이내인가(발자국 포함 판정용)."""
        if self._km is None:
            return False
        if self._mask_at(x, y):
            return True
        n = max(4, int(2 * math.pi * clearance / 0.03))
        for i in range(n):
            a = 2 * math.pi * i / n
            if self._mask_at(x + clearance * math.cos(a), y + clearance * math.sin(a)):
                return True
        return False

    def _in_mask_now(self):
        """★버그픽스(7/10 실주행): 중심점만 보면 '중심은 밖, 발자국만 안'일 때 예외가 안 걸려
        전진이 차단되고 로봇이 금지존 안에 141초 갇혔다. 발자국(반경) 기준으로 판정한다."""
        p = self._rs_pose()
        if p is None:
            return False
        r = float(self.get_parameter('mask_guard_radius').value)
        if self._mask_at(p[0], p[1]):
            return True
        for i in range(8):
            a = 2 * math.pi * i / 8
            if self._mask_at(p[0] + r * math.cos(a), p[1] + r * math.sin(a)):
                return True
        return False

    def _mask_seg_blocked(self, x0, y0, x1, y1, step=0.02):
        """직선 (x0,y0)->(x1,y1) 이 금지존 마스크를 지나는가. 로봇 반경만큼 부풀려 검사."""
        if self._km is None:
            return False
        r = float(self.get_parameter('mask_guard_radius').value)
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / step))
        for i in range(n + 1):
            t = i / n
            x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            if self._mask_at(x, y):
                return True
            for k in range(4):   # 발자국 4방향 (반경 r)
                a = math.pi / 2 * k
                if self._mask_at(x + r * math.cos(a), y + r * math.sin(a)):
                    return True
        return False

    def _mask_guard(self, v):
        """★금지존 마스크 하드가드 — 진행 방향 mask_guard_look[m] 안에 금지존이 있으면 0.

        ★적용 범위 = 이벤트 국면만(사용자 확정 7/10: "랩은 그동안 실패한 적 거의 없다.
          내가 말하는 건 이벤트 상황에 넘어가는 것").
          이유(실측): 순찰선 중심이 금지존 경계에서 최소 0.113m 인데 로봇 반경이 0.10m 라,
          랩에 발자국 가드를 걸면 12구간 중 11구간에서 상시 정지한다(기하학적으로 불가능).
          랩·도킹은 Nav2/검증된 서보가 그대로 돌고, 실제로 밟은 적도 없다.

        해당 국면: MOVING_TO_EVENT(마무리 중앙정렬·거리조정·BACKOFF) / ESCAPE(서보 끼임 탈출).
        (이벤트 '접근'은 Nav2가 하므로 코스트맵 KeepoutFilter가 이미 지킨다.)

        예외: 이미 마스크 안이면 통과(나가야 하므로). 회전(w)은 막지 않는다."""
        if abs(v) < 1e-4 or self._km is None:
            return v
        if not self.get_parameter('use_mask_hardguard').value:
            return v
        if not (self._rs_evt
                or self.current_state in (PatrolState.MOVING_TO_EVENT, PatrolState.ESCAPE)):
            return v          # 랩·도킹·RTC 등은 옛 동작 그대로
        if self._in_mask_now():
            self.get_logger().error('★금지존 안에 있음 — 탈출 위해 이동 허용',
                                    throttle_duration_sec=2.0)
            return v
        p = self._rs_pose()
        if p is None:
            return v
        look = float(self.get_parameter('mask_guard_look').value)
        sign = 1.0 if v > 0 else -1.0
        th = p[2] if sign > 0 else p[2] + math.pi
        x1 = p[0] + look * math.cos(th)
        y1 = p[1] + look * math.sin(th)
        if self._mask_seg_blocked(p[0], p[1], x1, y1):
            self.get_logger().error(
                f'★★금지존 하드가드: {"전진" if v > 0 else "후진"} 차단 '
                f'(상태={self.current_state.name}, {look:.2f}m 앞 금지존)',
                throttle_duration_sec=1.0)
            return 0.0
        return v

    def _move_guard(self, v):
        """★MOVE_GUARD(7/10, 사용자 원칙 "조금이라도 움직이면 코스트맵·라이다 세트"):
        속도를 내보내는 마지막 관문. 병진(v)이 향하는 쪽이 라이다든 금지존이든 막혀 있으면 0으로 깎는다.
        개별 가드를 하나씩 고치는 방식은 새 코드 경로가 생길 때마다 구멍이 났다(오늘 후진 금지존).

        ★적용 범위 = 이벤트 접근·마무리만(move_guard_scope='event', 기본). 사용자 확정 7/10:
          - 도킹/언도킹: 도킹존은 '빨강칸'(치명 셀)이라 가드를 켜면 스스로 진입을 막아 영영 못 들어간다.
          - 랩 순찰: 이미 잘 도는 검증된 루프 — 새 게이트로 좁은 통로 헛정지 위험. 손대지 않는다.
          (금지존을 밟은 건 이벤트 후진/마무리였다. 거기만 막으면 된다.)

        예외 둘 — ①ESCAPE: 금지존에서 빠져나오려 움직이는 중이라 막으면 영영 갇힌다.
                  ②이미 금지존 위: 모든 방향이 막힘이라 못 나간다.
        회전(w)은 막지 않는다 — 방향을 틀어야 탈출한다."""
        if abs(v) < 1e-4:
            return v
        scope = str(self.get_parameter('move_guard_scope').value).strip().lower()
        if scope == 'off':
            return v
        # 도킹 계열은 scope와 무관하게 항상 면제 — 켜면 도킹 자체가 불가능해진다.
        if self.current_state in (PatrolState.DOCKING, PatrolState.UNDOCKING,
                                  PatrolState.DOCK_DWELL, PatrolState.CHARGING):
            return v
        if self._rs_dock_retry:          # 프리도킹 정밀 재접근(서보) — 도킹존 접근 경로
            return v
        if scope == 'event' and not (
                self._rs_evt or self.current_state == PatrolState.MOVING_TO_EVENT
                or self._prox_active):    # ★PROXIMITY(7/13): 양보 후진도 가드 세트 적용
            return v                      # 랩 순찰 등은 옛 동작 그대로
        if self.current_state == PatrolState.ESCAPE or self._pose_in_keepout():
            return v
        sign = 1.0 if v > 0 else -1.0
        lidar = self._rs_front if v > 0 else self._rs_rear
        stop = RS_FRONT_STOP if v > 0 else 0.22
        # ★UNWEDGE_LIDAR(7/17, 사용자: "빠져나오는 게 쉽지 않다. 뒤로 가는 거리는 라이다에게
        #   맡기자 — cm 로 정하면 벽에 박을 수 있으니까"):
        #   실측 교착(로봇1 7/17): _mask_guard 는 발자국 기준(_in_mask_now)이라 '금지존 안 → 탈출
        #   허용'인데, 이 가드는 중심점 코스트만 보는 _pose_in_keepout 이라 '밖'으로 판정 →
        #   금지존 검사가 계속 살아서 후진을 막았다("★이동 차단(후진)" 반복 + 동시에
        #   "★금지존 안에 있음 — 탈출 위해 이동 허용" 이 찍힘 = 두 판정 불일치가 증거).
        #   → 여기서도 발자국 기준을 인정하되, **금지존 검사만 면제하고 라이다는 유지**한다.
        #     후진 거리를 상수로 박지 않는 이유 = 뒤가 벽이면 그 상수가 곧 충돌이다.
        #     라이다가 stop(0.22m) 까지 허용 → '갈 수 있는 만큼'만 자동으로 물러난다.
        if self.get_parameter('use_unwedge_lidar').value and self._in_mask_now():
            if lidar is not None and lidar < stop:
                self.get_logger().warn(
                    f'★끼임 탈출 차단({"전진" if v > 0 else "후진"}): 라이다 {lidar:.2f}m<{stop:.2f}m '
                    f'— 그쪽은 벽. 다른 방향 필요', throttle_duration_sec=2.0)
                return 0.0
            self.get_logger().warn(
                f'★끼임 탈출({"전진" if v > 0 else "후진"}): 금지존 검사 면제, 라이다만 적용 '
                f'(후방 {lidar if lidar is None else round(lidar, 2)}m — 갈 수 있는 만큼)',
                throttle_duration_sec=2.0)
            return v
        why, blk = self._blocked_dir(sign, lidar, stop, 0.30)
        if blk:
            # ★FRAME_SETTLE(7/13 실주행 교착): 가드 차단이 지속되는데 DRIVE 의 도착판정
            #   (_rs_cost_ahead)은 다른 셀을 봐서 안 울림 → 전진도 촬영도 없는 무한대기 실증
            #   (대상 0.43m 정중앙에서 정지 채 대기). 차단 지속시간을 기록해 DRIVE 가 읽는다.
            if v > 0:
                _n = self._rs_now_sec()
                if self._mg_block_first is None:
                    self._mg_block_first = _n
                self._mg_block_last = _n
            self.get_logger().warn(
                f'★이동 차단({"전진" if v > 0 else "후진"}): {why}',
                throttle_duration_sec=2.0)
            return 0.0
        if v > 0:
            self._mg_block_first = None   # 전진 뚫림 — 차단 연속성 리셋
        return v

    def _blocked_dir(self, sign, lidar_min, lidar_stop, keep_dist):
        """★KEEPOUT_SET(7/10): 방향별 통합 장애물 판정 = 라이다 OR 금지존.
        (사유 문자열, True/False) 반환 — 로그에 무엇이 막았는지 남긴다."""
        if lidar_min is not None and lidar_min < lidar_stop:
            return (f'라이다 {lidar_min:.2f}m<{lidar_stop:.2f}m', True)
        if self._keepout_dir(sign, keep_dist) is True:
            return (f'금지존/막힘 코스트({keep_dist:.2f}m 앞)', True)
        return ('', False)

    def _rs_cost_at(self, ax, ay):
        """(odom 프레임) 임의 점의 local costmap cost. 맵 밖/미수신이면 None."""
        if self._costmap is None:
            return None
        cm = self._costmap
        info = cm.info
        cx = int((ax - info.origin.position.x) / info.resolution)
        cy = int((ay - info.origin.position.y) / info.resolution)
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return cm.data[cy * info.width + cx]

    def _rs_cost_ahead(self, dist=0.18):
        """(odom 프레임) 진행방향 dist 앞 셀의 local costmap cost(keepout 포함). 못 읽으면 None."""
        if self._odom is None:
            return None
        ox, oy, oyaw = self._odom
        return self._rs_cost_at(ox + dist * math.cos(oyaw), oy + dist * math.sin(oyaw))

    def _rs_path_blocked(self, look=0.60, step=0.05):
        """★KEEPOUT_SERVO(7/9): 서보 주행은 Nav2를 안 거쳐 코스트맵(=KeepoutFilter)을 보지 않는다.
        지금까지 무사했던 건 웨이포인트가 금지존을 피해 찍혀 있었기 때문일 뿐이고,
        AMCL이 틀어지면 그대로 밟는다(실주행 목격). 전방 1셀(_rs_cost_ahead)만 보면 비스듬히
        스치는 궤적을 못 막으므로, 진행선을 따라 look[m]까지 step 간격으로 샘플링한다.
        막힌 지점의 거리(m)를 반환, 없으면 None."""
        if self._odom is None or self._costmap is None:
            return None
        block = self.get_parameter('escape_cost_block').value
        ox, oy, oyaw = self._odom
        d = step
        while d <= look:
            c = self._rs_cost_at(ox + d * math.cos(oyaw), oy + d * math.sin(oyaw))
            if c is not None and c >= block:
                return d
            d += step
        return None

    def _rs_evt_arrived(self):
        # ★이벤트 현장 도착 — Nav2 SUCCEEDED와 동일 처리(handle_goal_result 성공분기 복제).
        #   PAUSED 사유='EVENT_<type>' → id2_follow_detector의 yaw정렬 트리거 조건과 호환.
        self.get_logger().info('★ROUTE_SERVO 이벤트 현장 도착')
        self._rs_reset()
        self.retry_count = 0
        self.dispatch_retry_count = 0
        # ★CENTER_FALLBACK: 대상 방향 저장(중앙정렬 bearing 유실 시 개루프 폴백) — Nav2 분기와 동일
        if self.dispatch_target is not None:
            _q = self.dispatch_target.pose.orientation
            self._event_target_yaw = math.atan2(
                2 * (_q.w * _q.z + _q.x * _q.y),
                1 - 2 * (_q.y * _q.y + _q.z * _q.z))
        self.dispatch_target = None
        if getattr(self, 'dispatch_event_type', '') == 'HANDOVER':
            if 0 <= self.dispatch_target_wp < len(self.waypoints):
                self.current_waypoint_index = self.dispatch_target_wp
            else:
                self.get_logger().warn(
                    f'HANDOVER wp 인덱스 이상({self.dispatch_target_wp}) -> 0')
                self.current_waypoint_index = 0
            self.get_logger().info(
                f'HANDOVER 인계 완료 -> wp{self.current_waypoint_index}부터 순찰')
            self.change_state(PatrolState.PATROLLING)
        else:
            # ★SERVO_FINALIZE(7/13 실주행): 서보 도착이 마무리 체인을 건너뛰고 즉시 PAUSED →
            #   사진이 0.43m(60cm 미달)·수평트림 미실행(실측). Nav2 도착과 동일하게
            #   폐루프 마무리(중앙정렬→거리조정 60cm→평행트림→최종정렬)를 태운 뒤 PAUSED.
            self.get_logger().info('파견 현장 서보 도착 → 폐루프 마무리(정렬→60cm→수평) 시작')
            self._event_centering = True
            self._event_center_t0 = self._rs_now_sec()
            self._reacq_begin_episode()
            self._center_bmin = None

    def _rs_advance(self):
        self.retry_count = 0        # 구간 성공 = 진전 → Nav2 재시도 카운터 리셋(기존 성공처리와 동일)
        self._rs_seg_retries = 0
        self._cd_auto_detour_used = False   # ★CLEAR_DETOUR: 진전 → 자율우회 티켓 리셋(막힘 국면 종료)
        self.get_logger().info(f'ROUTE_SERVO {self._rs_label()} 완료')
        self._rs_leg += 1
        self.publish_state()        # ★WP_REPORT(7/13): 구간 진행마다 목표 노드 갱신 발행(관제 표시용)
        if self._rs_leg >= len(self._rs_full) - 1:
            self._rs_lap_complete()
        else:
            self._rs_enter_turn()

    def _event_early_stop(self, now):
        """★EARLY_STOP(7/10, 사용자 요구): "골에 가까이 가면서 객체를 찾으면 그 찾은 위치에서
        60cm 거리에 멈추고 가운데로 맞춘다."

        왜: 파견 goal 은 (서버 글로벌캠이든 우리 fusion 이든) 계산된 좌표라 오차가 있고,
        금지존 코앞에 찍히기도 한다(실측 3.4cm → 도착=발자국 침범). 반면 로봇이 대상을
        **직접 보고 있을 때**의 라이다 실거리는 오차가 없다.
        → 접근 중 대상이 화면 중앙(±early_stop_bearing_deg) 에 잡히고 전방 협각 라이다가
          event_stop_range 이하를 읽으면, 그 자리서 goal 을 끊고 마무리(중앙정렬→거리조정→촬영).
          글로벌캠 좌표는 '저쪽에 있다'는 안내로만 쓰고 최종 정지점은 우리가 정한다.

        True 반환 = 조기정지 발동(호출부는 더 진행하지 않음)."""
        if not self.get_parameter('use_event_early_stop').value:
            return False
        # ★RAW_GOAL(7/10): 좌표까지 가서 회전 탐색하는 모드에선 조기정지를 쓰지 않는다.
        #   (조기정지는 '대상 60cm 앞에 선다'는 standoff 규약의 일부다.)
        if self.get_parameter('use_raw_goal_event').value:
            return False
        if self.current_state != PatrolState.MOVING_TO_EVENT:
            return False
        if self._event_preface or self._event_centering:
            return False           # 선응시/마무리 중이면 이미 우리 몫
        # 조건 ①대상이 보인다(bearing 신선) — 사용자: "보이면"
        b = self._fresh_event_bearing(now)
        if b is None:
            return False
        tgt = float(self.get_parameter('event_stop_range').value)
        tol = float(self.get_parameter('event_range_tol').value)

        # 조건 ②대상까지 남은 거리가 60cm 이하
        #   두 자를 함께 쓴다:
        #     (a) 전방 협각(±8°) 라이다 실측 — 대상이 정면일 때 가장 정확
        #     (b) 로봇 pose ↔ 대상 원위치(map) — 대상이 옆에 있어 라이다가 못 볼 때
        d = None
        src = ''
        lid = self._rs_front_ctr
        if (lid is not None
                and abs(math.degrees(b))
                <= float(self.get_parameter('early_stop_bearing_deg').value)):
            d, src = lid, '라이다 실측'
        else:
            p = self._rs_pose()
            t = self._event_target_raw
            if p is not None and t is not None:
                d, src = math.hypot(t[0] - p[0], t[1] - p[1]), '대상좌표 거리'
        if d is None or d > tgt + tol:
            return False           # 아직 멀다 — 계속 접근
        self.get_logger().warn(
            f'★조기정지: 대상까지 {d:.2f}m ({src}) ≤ {tgt:.2f}+{tol:.2f}m, 방위 {math.degrees(b):+.1f}° '
            f'— goal 취소, 이 자리서 마무리(계산 좌표 대신 실측 사용)')
        self.cancel_current_goal()
        self._rs_stop()
        self.dispatch_retry_count = 0
        self._event_centering = True
        self._event_center_t0 = self._rs_now_sec()
        self._reacq_begin_episode()
        self._event_final_phase = None      # 중앙정렬부터 순서대로
        return True

    def _rs_tick(self):
        # 20Hz 주행 틱. 게이트ON + (PATROLLING=랩 | MOVING_TO_EVENT=이벤트 접근)에서만 활동.
        # ★ESTOP_HARD(7/9): 비상정지 동안 0속도를 20Hz로 계속 발행 — Nav2 goal 취소가
        #   비동기라 취소 완료 전까지 controller가 쏘는 잔여 cmd_vel을 덮어써 즉시 감속.
        #   늦게 accept돼 돌아온 goal 핸들도 여기서 재차 취소(취소 유실 대비).
        if self.current_state == PatrolState.EMERGENCY_STOP:
            self._rs_cmd(0.0, 0.0, ramp=False)
            if self.current_goal_handle is not None:
                self.cancel_current_goal()
            return
        # (아래는 정상 주행 — ESTOP 0속도는 위에서 반환. 별도 100Hz 타이머가 추가로 0을 박는다.)
        # ★UNWEDGE(7/10): 끼임 탈출은 서보가 직접 한다. Nav2 Spin/DriveOnHeading은 '끼어서 못 움직이는'
        #   그 컨트롤러가 수행하므로 실패한다(로그: 회전·전진이 4ms만에 '완료'). 실패 시에만 옛 경로로.
        if self.current_state == PatrolState.ESCAPE:
            # ★주의: 이 지점엔 아직 아래쪽 `now`가 없다 — 같은 함수(_rs_now_sec)로 구해 단위를 맞춘다.
            if self._servo_unwedge_tick(self._rs_now_sec()):
                return
        # ★EARLY_STOP(7/10): 대상을 실제로 보고 60cm에 닿으면 goal 을 끊고 그 자리서 마무리.
        if self._event_early_stop(self._rs_now_sec()):
            return
        # ★PREFACE(7/9): 파견 수락 직후 '즉시정지→이벤트 정면 응시→접근' 순서 보장.
        #   bbox 중앙 폐루프(/event/bearing)로 대상을 화면 중앙에 놓고 출발 — 좁은 화각(28.9°)
        #   에서 접근 내내 대상을 프레임 안에 유지(도착 중앙정렬 bearing 유실의 근본 대책).
        if self._event_preface and self.current_state == PatrolState.MOVING_TO_EVENT:
            self._preface_step()
            return
        # ★NAV2_EVENT(7/8): Nav2 이벤트 접근 도착 후 폐루프 중앙정렬은 여기서(20Hz) 수행.
        if self._event_centering:
            self._visual_center_step()
            return
        gate = bool(self.get_parameter('use_route_servo').value)
        # ★CLEAR_DETOUR: 우회(Nav2) 진행중엔 서보 휴면 — cmd_vel 이중발행(Nav2와 싸움) 방지
        lap_active = (gate and self.current_state == PatrolState.PATROLLING
                      and not self._cd_detour_active)
        # ★NAV2_EVENT(7/8): Nav2 이벤트 접근 모드면 서보는 이벤트 주행 안 함(Nav2가 담당)
        evt_active = (gate and self.current_state == PatrolState.MOVING_TO_EVENT
                      and self.dispatch_target is not None
                      and not self._nav2_event()   # ★HYBRID_FALLBACK: 우회 전환 시 서보 자동 휴면(아래 reset)
                      and not self._cd_detour_active)  # ★EVT_DETOUR(7/22): 이벤트 우회 중 서보 휴면
        # ★DETOUR_BACKOFF(7/22): 우회 goal 발사 전 후진 간격확보. _rs_cmd/_blocked_dir 가드
        #   세트(후방 라이다∪코스트맵∪금지존)를 그대로 타므로 뒤가 막히면 자동 종료.
        #   종료 조건: 전방 RS_FRONT_SLOW(0.40m) 확보 / 후방 막힘 / 6s 타임아웃 → goal 발사.
        #   전방 None(=range_min 미만 가능)은 '확보'로 안 침 — 타임아웃/후방막힘이 안전망.
        if self._cd_detour_active and self._cd_backoff_t0 is not None:
            _bk_now = self._rs_now_sec()
            # 후방 검사선 0.15m — 0.30 은 순찰선 옆 벽 치명셀을 스쳐 즉시 오탐 종료(7/22 실측).
            #   한 틱 6cm 이동 + 매 틱 재검사라 짧은 선분으로도 침범 전에 선다.
            why, blk = self._blocked_dir(-1.0, self._rs_rear, 0.22, 0.15)
            cleared = (self._rs_front is not None
                       and self._rs_front >= RS_FRONT_SLOW)
            if blk or cleared or (_bk_now - self._cd_backoff_t0 > 6.0):
                self._rs_stop()
                self._cd_backoff_t0 = None
                _r = ('전방 %.2fm 확보' % self._rs_front) if cleared else (
                    f'후방 막힘({why})' if blk else '타임아웃')
                self.get_logger().info(f'★DETOUR_BACKOFF 종료({_r}) → 우회 goal 발사')
                self._send_clear_detour()
            else:
                self._rs_cmd(-0.06, 0.0)
            return
        if not lap_active and not evt_active:
            # ★PROXIMITY(7/13): 양보측은 경보가 풀릴 때까지 천천히 후진(20Hz, 가드 통과 필수).
            #   _rs_cmd 가 라이다∪코스트맵∪마스크를 검사하므로 뒤가 막히면 자동 0속도.
            if (self._prox_active and self._prox_yield
                    and self.current_state == PatrolState.PAUSED):
                self._rs_cmd(float(self.get_parameter('proximity_yield_v').value), 0.0)
                return
            if self._rs_full is not None or self._rs_evt \
                    or self._rs_lv != 0.0 or self._rs_lw != 0.0:
                self._rs_stop()
                self._rs_reset()   # 이탈 → 복귀 시 최근접 노드부터
            return
        if evt_active and not self._rs_evt:
            # 이벤트 접근 미션 시작(랩 진행중이었으면 자동 폐기)
            # ★RESUME_SAVED(7/8): 폐기 전에 '가던 노드'(현 구간 목표)를 저장 → 이벤트 후 재개 때
            #   최근접(이벤트 위치)이 아니라 발견 당시 순찰 위치부터 남은 구간을 마저 돎.
            if self._rs_full is not None and self._rs_leg + 1 < len(self._rs_full):
                self._rs_resume_node = self._rs_full[self._rs_leg + 1]
            self._rs_stop()
            self._rs_reset()
            self._rs_evt = True
            self._rs_enter_turn()
            t = self.dispatch_target.pose.position
            self.get_logger().info(f'★ROUTE_SERVO 이벤트 접근 시작 → map({t.x:.2f},{t.y:.2f})')
        if lap_active and self._rs_full is None:
            self._rs_build_lap()
            return
        p = self._rs_pose()
        if p is None:
            self._rs_cmd(0.0, 0.0)   # TF 순간 유실 — 감속 정지 유지
            return
        if self._rs_evt:
            tx = self.dispatch_target.pose.position.x
            ty = self.dispatch_target.pose.position.y
        else:
            tx, ty = self._rs_target()
        now = self._rs_now_sec()

        if self._rs_phase == 'SETTLE':
            self._rs_cmd(0.0, 0.0, ramp=False)
            self._rs_settle -= 1
            if self._rs_settle <= 0:
                if self._rs_next_phase == 'DRIVE':
                    self._rs_enter_drive(math.hypot(tx - p[0], ty - p[1]))
                elif self._rs_next_phase == 'FACEYAW':
                    self._rs_phase = 'FACEYAW'
                    self._rs_phase_t0 = now
                    self._center_bmin = None   # ★VISUAL_CENTER: 발산감지 초기화
                elif self._rs_evt:   # 'ADVANCE' (이벤트 미션)
                    self._rs_evt_arrived()
                else:   # 'ADVANCE' (랩)
                    self._rs_advance()
            return

        if self._rs_phase == 'FACEYAW':
            # ★VISUAL_CENTER(7/8): 카메라 실시간 방위각 폐루프 중앙정렬(공용 헬퍼). 검출 유실 시 map-yaw 폴백.
            st = self._visual_center_ctrl(now, p, self._rs_phase_t0)
            if st == 'done':
                self._rs_enter_settle('ADVANCE')
                return
            if st == 'turning':
                return
            # ── st == 'fallback'(검출 유실) → 기존 개루프 map-yaw 폴백(대략 향해 도착) ──
            q = self.dispatch_target.pose.orientation
            gyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            err = self._norm(gyaw - p[2])
            if abs(err) < RS_TURN_TOL or now - self._rs_phase_t0 > RS_TURN_TIMEOUT:
                self._rs_enter_settle('ADVANCE')
                return
            w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
            if abs(w) < RS_TURN_MIN:
                w = math.copysign(RS_TURN_MIN, w)
            self._rs_cmd(0.0, w)
            return

        if self._rs_phase == 'TURN':
            err = self._norm(math.atan2(ty - p[1], tx - p[0]) - p[2])
            # ★BACKOFF(7/6): 이벤트 목표가 로봇 등뒤 가까이(=마커에 이미 근접, 표준거리 미달)면
            #   돌아서지 않고 후진으로 거리 확보 — 마커를 계속 정면에 두고 물러남("뒤로가서라도").
            if (self._rs_evt and math.hypot(tx - p[0], ty - p[1]) < 0.7
                    and abs(err) > math.radians(110)):
                self.get_logger().info(
                    f'ROUTE_SERVO 이벤트: 목표가 등뒤 {math.hypot(tx - p[0], ty - p[1]):.2f}m '
                    f'— 후진으로 거리 확보(BACKOFF)')
                self._rs_phase = 'BACKOFF'
                self._rs_phase_t0 = now
                return
            if abs(err) < RS_TURN_TOL:
                self._rs_enter_settle('DRIVE')
                return
            if now - self._rs_phase_t0 > RS_TURN_TIMEOUT:
                self._rs_seg_fail(f'TURN 타임아웃({RS_TURN_TIMEOUT:.0f}s)')
                return
            w = max(-RS_TURN_MAX, min(RS_TURN_MAX, RS_TURN_KP * err))
            if abs(w) < RS_TURN_MIN:
                w = math.copysign(RS_TURN_MIN, w)
            self._rs_cmd(0.0, w)
            return

        if self._rs_phase == 'BACKOFF':
            # 이벤트 전용: 목표(등뒤)까지 직선 후진. 후방 라이다 가드 + 도착/타임아웃 시 방향정렬.
            dist = math.hypot(tx - p[0], ty - p[1])
            arrive_tol = self.get_parameter('patrol_xy_tolerance').value
            if dist < arrive_tol:
                self._rs_enter_settle('FACEYAW')
                return
            # ★KEEPOUT_SET(7/10): 후진 가드 = 후방 라이다 ∪ 뒤쪽 금지존 코스트맵.
            why, blk = self._blocked_dir(-1.0, self._rs_rear, 0.22, 0.30)
            if blk:
                self.get_logger().warn(f'BACKOFF 후방 막힘({why}) — 여기까지 확보, 도착 처리')
                self._rs_enter_settle('FACEYAW')
                return
            if now - self._rs_phase_t0 > 15.0:
                self.get_logger().warn('BACKOFF 타임아웃 — 현 지점 도착 처리(무포기)')
                self._rs_enter_settle('FACEYAW')
                return
            self._rs_cmd(-0.06, 0.0)
            return

        if self._rs_phase == 'DRIVE':
            # ★FRONT_STOP_FAR(7/22): 노드까지 멀면 0.30, 노드 근처(도착권)는 기존 0.12.
            #   막힘 타이머 가동 중엔 +0.06 히스테리시스 — 정지점이 곧 경계라 라이다 노이즈
            #   1~2cm 로 막힘/뚫림이 튀며 rs_obstacle_wait_sec 타이머가 리셋되는 것을 방지.
            _stop = (RS_FRONT_STOP_FAR
                     if math.hypot(tx - p[0], ty - p[1]) > RS_FRONT_STOP_FAR + 0.15
                     else RS_FRONT_STOP)
            if self._cd_block_since is not None:
                _stop += 0.06
            if self._rs_front is not None and self._rs_front < _stop:
                # ★EVT_DETOUR(7/22, 사용자 요청): 이벤트 접근(서보) 중 전방 막힘도 우회.
                #   랩과 달리 서버 verdict 대기(OBSTACLE_WAITING) 없이 5s 확정 → 즉시 우회
                #   (긴급 파견 중이라 기다릴 이유가 없고, OBSTACLE_WAITING 계약은 순찰 스코프).
                #   우회 goal = 대상에서 event_stop_range(0.6m) 당긴 스탠드오프(대상 직행 goal 은
                #   7/17 'Nav2=대상 박음' 전과). 대상 코앞(스탠드오프+0.25 이내) 막힘은 우회
                #   무의미 — 기존 seg fail(재시도→마무리) 그대로. 상태는 MOVING_TO_EVENT 유지,
                #   도착 후 서보 이벤트 접근이 새 위치에서 자동 재초기화(TURN→정렬→60cm 컷).
                if (self._rs_evt
                        and self.get_parameter('use_clear_detour').value):
                    _ed = math.hypot(tx - p[0], ty - p[1])
                    _rng = float(self.get_parameter('event_stop_range').value)
                    if _ed > _rng + 0.25:
                        if self._cd_block_since is None:
                            self._cd_block_since = now
                            self.get_logger().warn(
                                f'이벤트 접근 전방 {self._rs_front:.2f}m 막힘 — 정지 유지, '
                                f'{self.get_parameter("rs_obstacle_wait_sec").value:.1f}s '
                                f'지속 시 우회(★EVT_DETOUR)')
                        self._rs_stop()
                        if (now - self._cd_block_since
                                >= self.get_parameter('rs_obstacle_wait_sec').value):
                            self._cd_target_xy = (tx - (tx - p[0]) / _ed * _rng,
                                                  ty - (ty - p[1]) / _ed * _rng)
                            self._cd_target_node = 'EVT_STANDOFF'
                            self._cd_fallback_node = None
                            self._cd_fallback_xy = None
                            self._cd_block_since = None
                            self.get_logger().warn(
                                '★EVT_DETOUR 전방 막힘 지속 → Nav2 우회(스탠드오프 '
                                f'({self._cd_target_xy[0]:.2f},{self._cd_target_xy[1]:.2f}))')
                            self._start_clear_detour(PatrolState.MOVING_TO_EVENT)
                        return
                # ★CLEAR_DETOUR(게이트 ON + 랩 주행만): 즉시 실패(_rs_seg_fail) 대신 정지 유지.
                #   막힘이 rs_obstacle_wait_sec초 연속되면 — 재시도 소진 전에 — OBSTACLE_WAITING
                #   전이(서버가 이 상태 보고 YOLO verdict 발행하는 계약). 그 전에 뚫리면 계속 주행.
                #   게이트 OFF/이벤트 접근은 아래 기존 급정지→재시도 경로 100% 그대로.
                if (not self._rs_evt
                        and self.get_parameter('use_clear_detour').value):
                    if self._cd_block_since is None:
                        self._cd_block_since = now
                        self.get_logger().warn(
                            f'ROUTE_SERVO 전방 {self._rs_front:.2f}m 막힘 — 정지 유지, '
                            f'{self.get_parameter("rs_obstacle_wait_sec").value:.1f}s '
                            f'지속 시 장애물 확정(★CLEAR_DETOUR)')
                    self._rs_stop()   # 기존 전방가드와 동일한 즉시 급정지(0.12m 가드 침범 방지)
                    if (now - self._cd_block_since
                            >= self.get_parameter('rs_obstacle_wait_sec').value):
                        # 막힌 구간의 '도착 노드'를 우회 목표로 기억한 뒤 서보 정리
                        self._cd_target_node = self._rs_full[self._rs_leg + 1]
                        self._cd_target_xy = self._rs_target()
                        # ★DYN_OBS(7/7): 우회 goal이 벽 인플레이션에 거부될 때 쓸 예비 목표
                        #   = '다음다음 노드'. 매 막힘마다 새로 기억 → 스테일 없음.
                        ni = self._rs_leg + 2
                        if ni < len(self._rs_full):
                            self._cd_fallback_node = self._rs_full[ni]
                            self._cd_fallback_xy = self._rs_pts[self._cd_fallback_node]
                        else:
                            self._cd_fallback_node = None
                            self._cd_fallback_xy = None
                        self._cd_block_since = None
                        self.get_logger().warn(
                            f'ROUTE_SERVO 전방 막힘 지속 → OBSTACLE_WAITING '
                            f'(막힌 구간 도착노드 {self._cd_target_node} 기억, 서버 판정 대기)')
                        self._rs_stop()
                        self._rs_reset()   # 복귀 시 최근접 노드부터 이어달리기(_rs_fresh=False 유지)
                        self.save_waypoint_index()
                        self.change_state(PatrolState.OBSTACLE_WAITING)
                    return
                self._rs_seg_fail(f'전방 {self._rs_front:.2f}m<{_stop:.2f} 급정지')
                return
            if self._cd_block_since is not None:
                self._cd_block_since = None   # ★CLEAR_DETOUR: 확정 전에 다시 뚫림 — 타이머 리셋
            dist = math.hypot(tx - p[0], ty - p[1])
            # ★금지존 가드(7/6, 이벤트 접근만): 진행방향 앞 셀이 금지존/막힘 코스트면
            #   거기가 최근접 안전점 — 침입하지 않고 그 자리서 도착 처리(마커는 yaw정렬로 바라봄).
            #   랩 주행엔 미적용(순찰선은 인플레이션 위를 지나는 게 정상이라 오탐 유발).
            if self._rs_evt:
                # ★SERVO_60CM(7/13, 사용자: "60cm보다 앞에 가서 찾고 있잖아"): raw goal 은 대상
                #   코앞이라 goal 까지 몰면 60cm 를 반드시 지나친다 — 보이는 대상 기준 60cm 에서
                #   접근을 끊는다(EARLY_STOP 의 서보판). 거리 이중판정:
                #   ①정면(±8°)에 잡혀 있으면 전방 라이다 실측 ②대상 원위치까지 map 거리 — 작은 쪽.
                _tgt_rng = float(self.get_parameter('event_stop_range').value)
                _dc = []
                _b = self._fresh_event_bearing(now)
                if (_b is not None and self._rs_front_ctr is not None
                        and abs(_b) <= math.radians(
                            float(self.get_parameter('early_stop_bearing_deg').value))):
                    _dc.append(self._rs_front_ctr)
                # ★PHOTO_SPOT(7/14): 서버 좌표=촬영지점이라 'map거리 60cm 컷'은 지점 60cm 앞
                #   조기도착(=이벤트서 1.2m)을 만든다 — 기본 OFF. 라이다 실측 컷은 유지.
                if (bool(self.get_parameter('use_map_range_cut').value)
                        and self._event_target_raw is not None):
                    _dc.append(math.hypot(self._event_target_raw[0] - p[0],
                                          self._event_target_raw[1] - p[1]))
                if _dc and min(_dc) <= _tgt_rng:
                    self.get_logger().info(
                        f'★서보 60cm 도달(실측 {min(_dc):.2f}m) — 접근 즉시 종료 → 마무리(정렬→60cm→수평)')
                    self._rs_stop()
                    self._rs_evt_arrived()
                    return
                c = self._rs_cost_ahead()
                # ★FRAME_SETTLE(7/13): 도착판정 셀(c)이 안 울려도, 전진 가드가 1.5s 이상
                #   연속 차단 중이면 같은 '막힘'이다 — 대상 정중앙에 두고 무한대기하던 교착 해소.
                _gb = (self._mg_block_first is not None
                       and self._mg_block_last is not None
                       and now - self._mg_block_last < 0.6
                       and self._mg_block_last - self._mg_block_first > 1.5)
                if _gb and c is None:
                    c = 100   # 가드 차단을 막힘 코스트로 승격(아래 분기 공용)
                if c is not None and (_gb or c >= self.get_parameter('escape_cost_block').value):
                    # ★HYBRID_FALLBACK(7/13): 멀리서 막히면 '여기가 최선' 포기 대신 Nav2 우회.
                    #   1.2m 밖 촬영 = 안면인식 불가(실측). 가까우면(≤1.0m) 종전대로 그 자리 촬영.
                    _lim = float(self.get_parameter('blocked_settle_max').value)
                    if dist > _lim and not self._event_force_nav2:
                        self.get_logger().warn(
                            f'이벤트 접근: 전방 셀 cost={c} 막힘 + 잔여 {dist:.2f}m>{_lim:.1f}m — '
                            f'서보 직선 한계 → 이 이벤트 Nav2 우회 전환')
                        self._event_force_nav2 = True
                        self._rs_cmd(0.0, 0.0, ramp=False)
                        # MTE 재진입(ESTOP 복귀와 동일 패턴): 공통부가 서보 이벤트 상태를 정리하고
                        # 상태루프가 _nav2_event()=True 를 보고 Nav2 goal 을 발사한다.
                        self.change_state(PatrolState.MOVING_TO_EVENT)
                        return
                    self.get_logger().warn(
                        f'이벤트 접근: 전방 셀 cost={c}(금지존/막힘) — 최근접 안전점 도착 처리'
                        f'(목표까지 잔여 {dist:.2f}m, 침입 안 함)')
                    self._rs_enter_settle('FACEYAW')
                    return
            # ★PHOTO_SPOT(7/14, 사용자: "진짜 그 좌표로"): 서버 좌표=최적 촬영지점이라
            #   느슨 도착(patrol_xy_tolerance)이면 구도가 어긋난다 — event_arrive_tol(0.05)로 조임.
            arrive_tol = (float(self.get_parameter('event_arrive_tol').value)
                          if self._rs_evt else RS_XY_TOL)
            if dist < arrive_tol:
                self._rs_enter_settle('FACEYAW' if self._rs_evt else 'ADVANCE')
                return
            if now - self._rs_phase_t0 > self._rs_drive_timeout:
                # ★PHOTO_SPOT(7/14): 2cm 판정은 AMCL 지터로 영원히 안 잡힐 수 있다 —
                #   이벤트 접근에서 잔여 0.15m 이내면 실패 대신 도착 처리(무포기 원칙).
                if self._rs_evt and dist < 0.15:
                    self.get_logger().warn(
                        f'DRIVE 타임아웃이지만 잔여 {dist:.2f}m<0.15 — 촬영지점 도착 처리')
                    self._rs_enter_settle('FACEYAW')
                    return
                self._rs_seg_fail(f'DRIVE 타임아웃({self._rs_drive_timeout:.0f}s)')
                return
            err = self._norm(math.atan2(ty - p[1], tx - p[0]) - p[2])
            # ★VISUAL_TRACK(7/9): 이벤트 접근은 '대상을 보면서' 간다.
            #   도킹의 ALIGN(bearing→0)+직진 구조를 그대로 이벤트에 적용 —
            #   /event/bearing(화면 중심 기준)을 조향에 물려 대상을 계속 화면 가운데 둔다.
            #   bearing 유실(0.5s) 시 odom 헤딩 조향으로 자동 폴백(무손상).
            #   ※'방향이탈 재조준' 가드는 odom err 기준 그대로 — 시각 폭주 방지용 안전망.
            vb = None
            if self._rs_evt and self.get_parameter('use_visual_event_tracking').value:
                vb = self._fresh_event_bearing(now)
            if abs(err) > RS_RETURN_ANG and dist > RS_NEAR_DIST and vb is None:
                self.get_logger().info(
                    f'ROUTE_SERVO {self._rs_label()} 방향이탈({math.degrees(err):.0f}°) — 재조준')
                self._rs_stop()
                self._rs_enter_turn()
                return
            v = RS_DRIVE_V_NEAR if dist < RS_NEAR_DIST else RS_DRIVE_V
            # ★EV_SLOW(7/15 사용자: "객체 접근은 천천히 — 팍팍 가면 인식을 놓친다"):
            #   이벤트 접근에만 속도상한. 검출 스트림(2Hz)·시각추종이 이동을 따라오게.
            if self._rs_evt:
                v = min(v, float(self.get_parameter('event_drive_v').value))
            # ★DYN_OBS(7/7): 전방 계단 감속(랩 주행만) — 급정지(0.12m) 전에 0.40m부터
            #   비례 감속. 지나가는 사람/이동체 충돌 여유 + 서버 YOLO 판정 시간 확보.
            #   비키면 다음 틱 자동 재가속(상태 전이 없음). 이벤트 접근은 기존 가드 유지.
            if (not self._rs_evt and self._rs_front is not None
                    and self._rs_front < RS_FRONT_SLOW):
                scale = max(0.4, (self._rs_front - RS_FRONT_STOP)
                            / (RS_FRONT_SLOW - RS_FRONT_STOP))
                v = min(v, RS_DRIVE_V * scale)
            # ★DET_SLOW(7/17, 사용자: "safety/detections 에 한 번이라도 결과 오면 속도를 줄여 접근"):
            #   검출 수신 ~ fusion 파견 확정 사이 구간이 비어 있었다(랩 0.10 그대로 질주 → bbox
            #   흔들림 → 좌표 부정확). 검출이 '비어있지 않게' 오는 동안만 상한을 건다.
            #   ※ detections=[] 하트비트가 9.75Hz 로 상시 오므로 '메시지 도착'이 아니라
            #     '배열 비어있지 않음' 이 조건이다(안 그러면 항상 감속).
            #   ※ 이벤트 접근(_rs_evt)은 이미 event_drive_v 상한이 있어 min() 으로 자연 합류.
            if self.get_parameter('use_detect_slow').value:
                hold = float(self.get_parameter('detect_slow_hold_sec').value)
                if self._rs_now_sec() - self._det_last_t <= hold:
                    v = min(v, float(self.get_parameter('detect_slow_v').value))
            if vb is not None:
                # 대상이 화면 중앙에서 벗어난 만큼 조향(centering_sign은 회전 부호 보정용)
                sign = float(self.get_parameter('centering_sign').value)
                steer = sign * vb
                self.get_logger().info(
                    f'이벤트 시각추종: 화면방위 {math.degrees(vb):+.1f}° → 조향 (odom오차 {math.degrees(err):+.0f}°)',
                    throttle_duration_sec=1.0)
            else:
                steer = err
            w = max(-RS_YAW_CLAMP, min(RS_YAW_CLAMP, RS_YAW_KP * steer))
            self._rs_cmd(v, w)
            return

        # phase 미정(방어) — 현재 구간 재조준
        self._rs_enter_turn()

    def run_state_loop(self):
        self._update_stuck_watchdog()   # ★WATCHDOG: 조용한 멈춤 감지 → 필요시 ESCAPE 강제
        s = self.current_state

        # ★BATT_IMMED(7/7, 정책 33%=무조건 복귀 / 감사픽스: PAUSED·MOVING_TO_EVENT·
        #   OBSTACLE_WAITING 사각 제거). 노이즈(±2% 널뜀 실측) 방어=15초 연속일 때만.
        #   서보 모드면 LOW_BATTERY가 RTC_SERVO 경로(루트→프리도킹→도킹→CHARGING)로 처리.
        if (s in (PatrolState.PATROLLING, PatrolState.MOVING_TO_EVENT,
                  PatrolState.PAUSED, PatrolState.OBSTACLE_WAITING)
                and not self._rtc_docking and not self._cd_detour_active):
            if self.current_battery < self.get_parameter('battery_threshold').value:
                if self._batt_low_since is None:
                    self._batt_low_since = self.get_clock().now()
                elif ((self.get_clock().now() - self._batt_low_since).nanoseconds / 1e9
                      >= 15.0):
                    self.get_logger().warn(
                        f'배터리 {self.current_battery:.1f}% 저하 15s 지속({s.name}) '
                        f'→ 즉시 충전 복귀(LOW_BATTERY)')
                    self._batt_low_since = None
                    self.cancel_current_goal()
                    self._rs_stop()
                    self.dispatch_target = None   # 이벤트 접근 중이었으면 임무 폐기(복귀 우선)
                    self.save_waypoint_index()
                    self.change_state(PatrolState.LOW_BATTERY)
                    return
            else:
                self._batt_low_since = None

        if s == PatrolState.IDLE:
            pass  # set_mode PATROL_START 대기

        elif s == PatrolState.LOCALIZING:
            self.change_state(PatrolState.PATROLLING)

        elif s == PatrolState.UNDOCKING:
            # ★UNDOCK: 도킹존(0,0) 빨강칸을 전진으로 빠져나온 뒤 순찰 시작 (모션·정지 Pi 로컬).
            now = self.get_clock().now()
            if not self.undock_cmd_sent:
                self.dock_cmd_pub.publish(String(data='UNDOCK'))
                self.undock_cmd_sent = True
                self.undock_start = now
                self._dock_done_msg = None
                self.get_logger().info('UNDOCK 명령 발행 → Pi 전진 undock 대기')
                return
            if self._dock_done_msg == 'UNDOCK_DONE':
                self._dock_done_msg = None
                self.undock_cmd_sent = False
                self._rs_dock_retry = False   # ★ROUTE_SERVO: 새 랩 시작 — 종점 근처라도 랩 생략 금지
                self._rs_fresh = True         # ★ROUTE_SERVO: 언도킹 후엔 전체 랩
                # ★IDLE_DISPATCH(7/7): 대기 중 파견으로 언도크했으면 순찰 대신 임무로 직행
                if self._idle_dispatch_pending:
                    self._idle_dispatch_pending = False
                    self.get_logger().info('UNDOCK 완료 → 파견 임무 출동 (MOVING_TO_EVENT)')
                    self.change_state(PatrolState.MOVING_TO_EVENT)
                    return
                self.get_logger().info('UNDOCK 완료 → 순찰 시작')
                self.change_state(PatrolState.PATROLLING)
                return
            # 페일세이프: 타임아웃 시 순찰 강행(Pi 노드 확인 필요 — 옛 ESCAPE가 최후 보루)
            if (now - self.undock_start).nanoseconds / 1e9 >= self.get_parameter('undock_timeout').value:
                self.get_logger().warn(
                    'UNDOCK 타임아웃 — Pi 실행노드 확인 필요. 순찰 강행(빨강칸이면 ESCAPE 복구)')
                self._dock_done_msg = None
                self.undock_cmd_sent = False
                if self._idle_dispatch_pending:   # ★IDLE_DISPATCH: 타임아웃이어도 임무 강행
                    self._idle_dispatch_pending = False
                    self.change_state(PatrolState.MOVING_TO_EVENT)
                    return
                self.change_state(PatrolState.PATROLLING)

        elif s == PatrolState.PATROLLING:
            if not self.waypoints:
                self.get_logger().error('No waypoints loaded')
                self.change_state(PatrolState.STUCK)
                return
            # ★OBSTACLE_BRIDGE: 라이다가 전방을 막아 STOP이 enter_debounce초 지속되면 정지.
            #   collision_monitor가 안 떠 있으면 cm_stop_since가 늘 None이라 기존 순찰과 동일하게 동작.
            if self.cm_stop_since is not None:
                held = (self.get_clock().now() - self.cm_stop_since).nanoseconds / 1e9
                if held >= self.get_parameter('obstacle_enter_debounce').value:
                    self.get_logger().warn(
                        '전방 장애물 감지(collision_monitor STOP) -> OBSTACLE_WAITING')
                    self.cancel_current_goal()    # 굴러가던 goal 정리(재개 시 재발사)
                    self.save_waypoint_index()    # 재개용 wp 저장
                    self.change_state(PatrolState.OBSTACLE_WAITING)
                    return
            # ★ROUTE_SERVO 게이트 ON: 주행은 _rs_tick(20Hz)이 전담 — Nav2 goal 안 보냄.
            #   (장애물 debounce는 위에서 기존 그대로 동작. 상태가 바뀌면 서보는 자동 정지.)
            if self.get_parameter('use_route_servo').value:
                # ★CLEAR_DETOUR: 우회 진행중엔 Nav2가 주행 — goal이 유실됐으면
                #   (send 시점 Nav2 서버 미준비 등) 여기서 재발사.
                if (self._cd_detour_active and not self.goal_in_flight
                        and self._cd_target_xy is not None
                        and self._cd_backoff_t0 is None):   # ★DETOUR_BACKOFF: 후진 중엔 발사 보류
                    self._send_clear_detour()
                return
            if not self.goal_in_flight:
                # ★DOCK_TOL/DOCK_YAW: 마지막 wp(=충전존)면 xy·yaw 정밀, 아니면 느슨하게(교착방지)
                is_dock = (self.current_waypoint_index == len(self.waypoints) - 1)
                if is_dock:
                    self.set_goal_tolerance(
                        self.get_parameter('dock_xy_tolerance').value,
                        self.get_parameter('dock_yaw_tolerance').value)
                else:
                    self.set_goal_tolerance(
                        self.get_parameter('patrol_xy_tolerance').value,
                        self.get_parameter('patrol_yaw_tolerance').value)
                self.send_nav_goal(
                    self.build_pose_from_waypoint(
                        self.waypoints[self.current_waypoint_index]))

        elif s == PatrolState.ARRIVED:
            is_predock = (self.current_waypoint_index == len(self.waypoints) - 1)
            threshold = self.get_parameter('battery_threshold').value
            if is_predock:
                # ★DOCK_FIX: 마지막 wp = pre-dock(0.4,0)일 뿐, 아직 도킹존(0,0) 아님.
                #   (구코드는 여기서 바로 '도킹존 도착' 처리 → pre-dock 오판으로 handover/CHARGING
                #    조기발동 + initialpose(0,0,0)를 pre-dock(0.4,0)서 주입해 40cm 점프로 AMCL 교란.)
                if self.get_parameter('use_dock_executor').value:
                    # 게이트 ON: 마커 SEARCH→ALIGN→REVERSE(pi_dock)로 실제 (0,0) 진입 → DOCKING
                    # ★DOCK_STALE_FIX(7/9): 직전 실패 시도의 잔재(dock_cmd_sent=True/낡은 타이머)
                    #   때문에 DOCK 명령 미발행+유령 타임아웃(실주행 실증) → 진입 시 초기화
                    self.dock_cmd_sent = False
                    self._dock_done_msg = None
                    self.get_logger().info('pre-dock 도착 → DOCKING (마커 정렬+후진으로 도킹존 진입)')
                    self.change_state(PatrolState.DOCKING)
                else:
                    # 게이트 OFF(안전 기본): 도킹 미사용 → 랩 완료로 보고 다음 바퀴 계속.
                    #   ※ initialpose(0,0,0) 주입 안 함(로봇=pre-dock 0.4,0 → 주입 시 AMCL 40cm 점프).
                    #     handover/CHARGING도 안 함(오판 제거). = 깨끗한 연속 순찰.
                    self.get_logger().info('pre-dock 도착 = 랩 완료 → 다음 바퀴 계속(도킹 미사용)')
                    self.dock_arrival_count += 1
                    self.current_waypoint_index = \
                        (self.current_waypoint_index + 1) % len(self.waypoints)
                    if self.current_battery < threshold:
                        self.get_logger().warn(
                            f'배터리 {self.current_battery:.1f}% < {threshold}% → LOW_BATTERY (충전 복귀)')
                        self.save_waypoint_index()
                        self.change_state(PatrolState.LOW_BATTERY)
                    else:
                        self.change_state(PatrolState.PATROLLING)
            else:
                # 일반 wp 도착: 다음 wp + 배터리 체크(기존)
                self.current_waypoint_index = \
                    (self.current_waypoint_index + 1) % len(self.waypoints)
                if self.current_battery < threshold:
                    self.get_logger().warn(
                        f'배터리 {self.current_battery:.1f}% < {threshold}% → LOW_BATTERY (충전 복귀)')
                    self.save_waypoint_index()
                    self.change_state(PatrolState.LOW_BATTERY)
                else:
                    self.change_state(PatrolState.PATROLLING)

        elif s == PatrolState.DOCKING:
            # ★DOCK: pre-dock 도착 후 도킹존(0,0) 진입 확인 시 도착 확정.
            if not self.get_parameter('use_dock_executor').value:
                # 게이트 OFF(옛 #1 동작): DOCK 명령 안 보냄, (0,0) 진입만 감지. STUCK 안 감.
                if self._at_dock_zone():
                    self.get_logger().info('도킹존(0,0) 진입 확인 → 도킹존 도착 확정')
                    self._on_dock_zone_arrived()
                return
            # 게이트 ON: Pi에 후진도킹(pi_dock) 명령 → DOCK_DONE(주)/(0,0)진입(보조) 시 확정.
            now = self.get_clock().now()
            # ★DOCK_AIM(7/15 사용자 제안 "한바퀴 다 안 돌고 마커 쪽 보고 찾기"): DOCK 명령 전에
            #   believed 위치→도크 좌표 방향으로 선조준 회전. 마커가 화각(±14.5°) 안에 들어온 채
            #   SEARCH가 시작돼 몇 초 만에 락 — 맹회전 105s 제거. 조준 실패(12s)면 기존동작 폴백.
            if (self.get_parameter('use_dock_aim').value and not self.dock_cmd_sent):
                _p = self._rs_pose()
                if _p is not None:
                    if self._dock_aim_start is None:
                        self._dock_aim_start = now
                    _dx = float(self.get_parameter('dock_x').value) - _p[0]
                    _dy = float(self.get_parameter('dock_y').value) - _p[1]
                    _err = self._norm(math.atan2(_dy, _dx) - _p[2])
                    _el_aim = (now - self._dock_aim_start).nanoseconds / 1e9
                    if abs(_err) > math.radians(8.0) and _el_aim < 12.0:
                        _w = max(-0.5, min(0.5, 1.2 * _err))
                        if abs(_w) < 0.12:
                            _w = math.copysign(0.12, _w)
                        self._rs_cmd(0.0, _w)
                        self.get_logger().info(
                            f'★DOCK_AIM 마커방향 조준 잔여 {math.degrees(_err):+.0f}°',
                            throttle_duration_sec=1.0)
                        return
                    self._rs_stop()
                    self.get_logger().info(
                        f'★DOCK_AIM 조준 완료(잔여 {math.degrees(_err):+.1f}°, {_el_aim:.1f}s) → DOCK')
            if not self.dock_cmd_sent:
                self._dock_aim_start = None
                self.dock_cmd_pub.publish(String(data='DOCK'))
                self.dock_cmd_sent = True
                self.docking_start = now
                self._dock_done_msg = None
                self.get_logger().info('DOCK 명령 발행 → Pi 후진도킹(pi_dock) 대기')
                return
            if self._dock_done_msg == 'DOCK_DONE' or self._at_dock_zone():
                self.get_logger().info('후진도킹 완료(신호/위치) → 도킹존 도착 확정')
                self._dock_done_msg = None
                self.dock_cmd_sent = False
                self._on_dock_zone_arrived()
                return
            # ★DOCK_FAIL_FIX(7/9): Pi가 실패를 알려왔는데(SEARCH 마커 못찾음 등) 기존 코드는
            #   무시하고 dock_timeout까지 방치(실주행 실증: 41s 실패 후 109s 헛대기).
            #   즉시 STUCK 재시도(프리도킹 정밀 재접근 → 도킹 재시도)로 전환.
            if self._dock_done_msg == 'DOCK_FAIL':
                self.get_logger().error(
                    'Pi DOCK_FAIL 수신 — 즉시 재시도 경로(프리도킹 재접근 후 도킹)로 전환')
                self._dock_done_msg = None
                self.dock_cmd_sent = False
                self._rs_dock_retry = True
                self.change_state(PatrolState.STUCK)
                return
            # 페일세이프: 타임아웃 → 절대 가짜 도착처리 안 함(handover/충전 오발동 방지) → STUCK(사람확인)
            if (now - self.docking_start).nanoseconds / 1e9 >= self.get_parameter('dock_timeout').value:
                self.get_logger().error(
                    'DOCK 타임아웃 — 후진도킹 미완. 도착 미확정 → STUCK (단자/정렬 확인 필요)')
                self.dock_cmd_sent = False
                self._rs_dock_retry = True   # ★ROUTE_SERVO: 재시도 시 랩 생략하고 바로 도킹 재시도
                self.change_state(PatrolState.STUCK)

        elif s == PatrolState.MOVING_TO_EVENT:
            # ★NAV2_EVENT(7/8): 이벤트 접근을 Nav2 경로계획으로(금지존/장애물 회피). standoff 60cm라
            #   목표가 물체 60cm 앞 열린공간 → Nav2 goal 수락(구 "벽붙은 목표 거부" 문제 없음).
            #   도착(goal SUCCEEDED)하면 _event_centering=True 세팅 → _rs_tick이 폐루프 중앙정렬 → PAUSED.
            if self._nav2_event():
                if self._event_preface:
                    return   # ★PREFACE: 선응시 완료 전 접근 goal 발사 보류(_rs_tick이 수행)
                if self._event_centering:
                    return   # 중앙정렬은 _rs_tick(20Hz)이 수행
                if self.dispatch_target is not None and not self.goal_in_flight:
                    # 도착오차 느슨(초정밀은 도착 후 카메라 중앙정렬이 담당) — RPP 교착 방지
                    self.set_goal_tolerance(
                        self.get_parameter('patrol_xy_tolerance').value,
                        self.get_parameter('patrol_yaw_tolerance').value)
                    self.send_nav_goal(self.dispatch_target)
                return
            # ★ROUTE_SERVO(7/6): use_route_servo면 이벤트 접근도 서보(_rs_tick)가 직진 수행(구 경로).
            if self.get_parameter('use_route_servo').value:
                # ★EVT_DETOUR(7/22): 우회 goal 유실(Nav2 서버 미준비 등) 시 재발사 — 랩의
                #   PATROLLING 재발사와 동일 역할. 백오프 중엔 보류.
                if (self._cd_detour_active and not self.goal_in_flight
                        and self._cd_target_xy is not None
                        and self._cd_backoff_t0 is None):
                    self._send_clear_detour()
                return
            if self.dispatch_target is not None and not self.goal_in_flight:
                # (Nav2 경로) 출발 전 자기 셀이 막힘 수준이면 ESCAPE로 빼고 재발사
                if self._in_red():
                    self.get_logger().warn(
                        '파견 출발점이 막힘 코스트 → ESCAPE로 빼고 재발사')
                    self._enter_escape(PatrolState.MOVING_TO_EVENT)
                    return
                self.send_nav_goal(self.dispatch_target)

        elif s == PatrolState.LOW_BATTERY:
            # ★RTC_NAV2(7/8): 저배터리 자동복귀도 RTC와 동일하게 Nav2 최단경로로 프리도킹까지 이동
            #   → DOCKING(마커 후진) → CHARGING. (구 서보경유 폐기)
            self._rtc_docking = True
            # ★BATTERY Phase 2: 프리도킹으로 goal 발사 → RETURNING_TO_CHARGER.
            if self.charging_station is None:
                self.get_logger().error('충전소 좌표 없음 — 복귀 불가 (waypoints.yaml 확인)')
                return
            self.get_logger().info('LOW_BATTERY → 충전소로 복귀 시작 (RETURNING_TO_CHARGER, Nav2 최단경로)')
            self.set_goal_tolerance(                        # ★RTC_NAV2: 느슨 도착(초정밀은 마커 도킹) — 교착방지
                self.get_parameter('patrol_xy_tolerance').value,
                self.get_parameter('patrol_yaw_tolerance').value)
            self.send_nav_goal(self.build_pose_from_waypoint(self._rtc_target_wp()))
            self.change_state(PatrolState.RETURNING_TO_CHARGER)

        elif s == PatrolState.RETURNING_TO_CHARGER:
            # ★BATTERY Phase 2: 프리도킹으로 이동 중. goal 안 떠 있으면 재발사.
            if not self.goal_in_flight and self.charging_station is not None:
                self.set_goal_tolerance(                    # ★RTC_NAV2(7/8): 느슨 도착(초정밀은 마커 도킹) — 교착방지
                    self.get_parameter('patrol_xy_tolerance').value,
                    self.get_parameter('patrol_yaw_tolerance').value)
                self.send_nav_goal(self.build_pose_from_waypoint(self._rtc_target_wp()))

        elif s == PatrolState.CHARGING:
            # ★CHARGE_FIX: 충전 완료는 '배터리가 실제로 올라 target 도달'할 때만.
            #   배터리가 안 오르면(단자 미연결) 블라인드 타임아웃 강제완료 하지 않고,
            #   경고 후 계속 대기 → 방전된 로봇을 순찰로 내보내지 않음(2호기가 순찰 커버).
            now = self.get_clock().now()
            if self.charging_start is None:
                self.charging_start = now
                self.charging_ref_battery = self.current_battery   # 상승 감시 기준값
                self.charging_ref_time = now
                self.get_logger().info(
                    f'CHARGING 시작 — 배터리 {self.current_battery:.1f}% (실제 상승 감시)')
            target = self.get_parameter('charge_target_pct').value
            delta = self.get_parameter('charge_progress_delta').value
            stall = self.get_parameter('charge_stall_timeout').value
            # ① 완료: 배터리가 실제로 target 이상까지 올라옴
            if self.current_battery >= target:
                self.charging_start = None
                if (self._handover_sent
                        or not bool(self.get_parameter('auto_resume_after_charge').value)):
                    # ★감사픽스(이중순찰 방지) + ★STAY_DOCKED(7/14): 자동 재출격 금지 —
                    #   충전완료 후 IDLE 대기. 재투입은 관제 PATROL_START/파견으로만.
                    self.get_logger().info(
                        f'충전완료 ({self.current_battery:.1f}% ≥ {target}%) '
                        f'→ IDLE 대기(자동 재출격 안 함 — 관제 출발 신호 대기)')
                    self._handover_sent = False
                    self.change_state(PatrolState.IDLE)
                else:
                    self.get_logger().info(
                        f'충전완료 (배터리 {self.current_battery:.1f}% ≥ {target}%) '
                        f'→ RESUMING_AFTER_CHARGE')
                    self.change_state(PatrolState.RESUMING_AFTER_CHARGE)
            # ② 상승 감지: 기준값보다 delta 이상 오름 = 실제 충전중 → 기준 갱신(정체 타이머 리셋)
            elif self.current_battery >= self.charging_ref_battery + delta:
                self.get_logger().info(
                    f'충전 진행중 — {self.charging_ref_battery:.1f}% → '
                    f'{self.current_battery:.1f}% (target {target}%)')
                self.charging_ref_battery = self.current_battery
                self.charging_ref_time = now
            # ③ 정체 감지: stall초 동안 delta만큼도 안 오름 = 충전 안 됨 → 경고+대기(내보내지 않음)
            elif (now - self.charging_ref_time).nanoseconds / 1e9 >= stall:
                self.get_logger().warn(
                    f'충전 안 됨 — {stall:.0f}s간 배터리 안 오름 '
                    f'(현재 {self.current_battery:.1f}%). 충전 단자 연결 확인! (계속 대기)')
                self.charging_ref_time = now   # 경고 주기 리셋(로그 스팸 방지)
            # 그 외: 계속 대기 (배터리 실제로 오르길 기다림)

        elif s == PatrolState.RESUMING_AFTER_CHARGE:
            # ★BATTERY Phase 3: 충전 후 저장된 wp부터 순찰 재개.
            self.current_waypoint_index = self.saved_waypoint_index
            self.get_logger().info(
                f'충전 후 복귀 — wp{self.current_waypoint_index}부터 순찰 재개(executor면 undock 먼저)')
            # ★UNDOCK(P2): 충전은 도킹존(0,0)에서 → 재개 전 전진 undock(게이트 반영)
            self.change_state(self._post_dock_state())

        elif s == PatrolState.RETRYING:
            if self.retry_wait_start is None:
                self.retry_wait_start = self.get_clock().now()
                return
            elapsed = (self.get_clock().now()
                       - self.retry_wait_start).nanoseconds / 1e9
            if elapsed < RETRY_WAIT:
                return
            self.retry_wait_start = None
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Retry {self.retry_count}/{self.max_retries} (대기 후)')
                self.change_state(PatrolState.PATROLLING)
            else:
                self.get_logger().error('Max retries exceeded -> ESCAPE(탈출 시도)')
                self._enter_escape(PatrolState.PATROLLING)

        elif s == PatrolState.OBSTACLE_WAITING:
            now = self.get_clock().now()
            if self.obstacle_wait_start is None:
                self.obstacle_wait_start = now
            # ★OBSTACLE_BRIDGE ①: 전방이 다시 뚫림(비STOP이 clear_debounce초 지속) → 서버 없이 자동 재개.
            #   지나가던 사람 등 '동적 장애물 통과' 처리. 정적 장애물(쓰러진 사람 등)은 안 뚫려서 ②로 감.
            if self.cm_clear_since is not None:
                clear_for = (now - self.cm_clear_since).nanoseconds / 1e9
                if clear_for >= self.get_parameter('obstacle_clear_debounce').value:
                    self.get_logger().info('전방 다시 뚫림 -> 순찰 재개 (동적 장애물 통과)')
                    self.obstacle_wait_start = None
                    self.change_state(PatrolState.PATROLLING)
                    return
            # ★CLEAR_DETOUR ①': 서보 모드는 cm 브리지 비활성(6/28) — 서보 전방 라이다로
            #   뚫림을 감지해 자동 재개(동적 장애물 통과). 게이트 OFF면 기존과 완전 동일.
            if (self.get_parameter('use_route_servo').value
                    and self.get_parameter('use_clear_detour').value):
                if self._rs_front is not None and self._rs_front > CD_FRONT_CLEAR:
                    if self._cd_clear_since is None:
                        self._cd_clear_since = now
                    elif ((now - self._cd_clear_since).nanoseconds / 1e9
                          >= self.get_parameter('obstacle_clear_debounce').value):
                        self.get_logger().info(
                            '전방 다시 뚫림(서보 라이다) -> 순찰 재개 (동적 장애물 통과)')
                        self._cd_clear_since = None
                        self.obstacle_wait_start = None
                        self._cd_auto_detour_used = False   # 뚫림 = 막힘 국면 종료 → 티켓 리셋
                        self.change_state(PatrolState.PATROLLING)
                        return
                else:
                    self._cd_clear_since = None
            # ② verdict 무응답 타임아웃 -> STUCK (서버 판정도, 뚫림도 없을 때)
            elapsed = (now - self.obstacle_wait_start).nanoseconds / 1e9
            if elapsed > self.get_parameter('obstacle_verdict_timeout').value:
                # ★CLEAR_DETOUR: 서보+게이트 맥락 + 우회목표 기억돼 있으면 STUCK 대신
                #   '자율 우회' 1회 — 서버 무응답이어도 순찰 지속. 우회 거부/실패는 기존
                #   RETRYING(→소진 시 ESCAPE) 폴백 그대로 = STUCK/RETRYING은 최후 수단.
                #   같은 막힘에서 재무응답(_cd_auto_detour_used)이면 기존 STUCK(무한루프 방지).
                #   구 cm 브리지 경로(서보 아님/목표 없음)는 기존 STUCK 무손상.
                if (self.get_parameter('use_route_servo').value
                        and self.get_parameter('use_clear_detour').value
                        and self._cd_target_xy is not None
                        and not self._cd_auto_detour_used):
                    self._cd_auto_detour_used = True
                    self.get_logger().warn(
                        f'verdict {self.get_parameter("obstacle_verdict_timeout").value:.0f}s '
                        f'무응답 — 자율 우회 시도(도착노드 {self._cd_target_node}, '
                        f'관제 기록용 OBSTACLE_WAITING 상태는 이미 발행됨)')
                    self._start_clear_detour()
                    return
                self.get_logger().error('verdict 무응답 타임아웃 -> STUCK')
                self.obstacle_wait_start = None
                self.stuck_retry_count = 0
                self.change_state(PatrolState.STUCK)

        elif s == PatrolState.RESUMING:
            # 저장된 wp부터 순찰 재개 (PAUSED → RESUME 경로)
            self.current_waypoint_index = self.saved_waypoint_index
            self.change_state(PatrolState.PATROLLING)

        elif s == PatrolState.PAUSED:
            pass  # 관제 RESUME(set_mode) 무한 대기

        elif s == PatrolState.MANUAL_CONTROL:
            pass  # 서버 cmd_vel 직접 제어 (MANUAL_EXIT 대기)

        elif s == PatrolState.STUCK:
            now = self.get_clock().now()
            if self.stuck_wait_start is None:
                self.stuck_wait_start = now
                self.get_logger().warn(
                    f'STUCK 진입 — {self.stuck_retry_wait}s 대기 후 재시도')
            else:
                elapsed = (now - self.stuck_wait_start).nanoseconds / 1e9
                if elapsed >= self.stuck_retry_wait:
                    self.stuck_wait_start = None
                    self.retry_count = 0
                    if self.stuck_retry_count < self.stuck_max_retries:
                        self.stuck_retry_count += 1
                        self.get_logger().info(
                            f'STUCK 재시도 {self.stuck_retry_count}/'
                            f'{self.stuck_max_retries} (wp{self.current_waypoint_index})')
                        self.change_state(PatrolState.PATROLLING)
                    else:
                        self.stuck_retry_count = 0
                        skipped = self.current_waypoint_index
                        self.current_waypoint_index = \
                            (self.current_waypoint_index + 1) % len(self.waypoints)
                        self.get_logger().error(
                            f'STUCK 재시도 소진 — wp{skipped} 스킵 '
                            f'→ wp{self.current_waypoint_index}')
                        self.change_state(PatrolState.PATROLLING)

        elif s == PatrolState.ESCAPE:
            self._run_escape()

        elif s == PatrolState.DOCK_DWELL:
            # ★DWELL: 도킹존에서 dock_dwell_sec초 대기 후 순찰 재개
            now = self.get_clock().now()
            if self.dock_dwell_start is None:
                self.dock_dwell_start = now
                self.get_logger().info(
                    f'도킹존 대기 시작 ({self.get_parameter("dock_dwell_sec").value}s)')
            else:
                elapsed = (now - self.dock_dwell_start).nanoseconds / 1e9
                if elapsed >= self.get_parameter('dock_dwell_sec').value:
                    self.dock_dwell_start = None
                    self.get_logger().info('도킹존 대기 완료 → 순찰 재개(executor면 undock 먼저)')
                    self.change_state(self._post_dock_state())

        elif s == PatrolState.EMERGENCY_STOP:
            # ★ESTOP_HOLD(7/15 사용자: "이멀전시 스탑 즉시 멈추도록"): 0속도를 상태 루프마다
            #   지속 발행 — 정지 명령이 WiFi 유실로 안 닿아 미끄러지는 것 차단(로봇2 지연 실측).
            #   단발 정지(_rs_stop)는 패킷 1개에 목숨 걸던 구조였다.
            self._rs_stop()

        # 타이머 리셋 (해당 상태 벗어나면 초기화)
        if s != PatrolState.OBSTACLE_WAITING:
            self.obstacle_wait_start = None
            self._cd_clear_since = None                    # ★CLEAR_DETOUR: 뚫림 타이머 리셋
            if not self._cd_detour_active:
                self._cd_target_xy = None                  # ★CLEAR_DETOUR: 우회중 아니면 목표 파기(스테일 방지)
                self._cd_target_node = None
        if self._cd_detour_active and s not in (PatrolState.PATROLLING,
                                                PatrolState.MOVING_TO_EVENT):
            # ★CLEAR_DETOUR: 우회 중 다른 상태로 이탈(ESTOP/파견/ESCAPE 등, goal은 각 경로가
            #   이미 취소) → 우회 폐기. 남겨두면 나중 복귀 때 스테일 재발사 위험.
            #   ★EVT_DETOUR(7/22): MOVING_TO_EVENT 는 이벤트 우회의 정상 상태 — 폐기 제외.
            self.get_logger().info('★CLEAR_DETOUR 우회 중 상태 이탈 — 우회 폐기(서보가 재개 담당)')
            self._cd_detour_active = False
            self._cd_backoff_t0 = None                     # ★DETOUR_BACKOFF: 후진도 폐기
            self._cd_target_xy = None
            self._cd_target_node = None
        if s != PatrolState.STUCK:
            self.stuck_wait_start = None
        if s != PatrolState.CHARGING:          # ★BATTERY Phase3: CHARGING 벗어나면 충전타이머 리셋
            self.charging_start = None
        if s != PatrolState.DOCK_DWELL:        # ★DWELL: 도킹대기 벗어나면 타이머 리셋
            self.dock_dwell_start = None

    def save_waypoint_index(self):
        self.saved_waypoint_index = self.current_waypoint_index


def main(args=None):
    rclpy.init(args=args)
    node = PatrolCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()