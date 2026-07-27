#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pi_dock_executor.py — Pi 로컬 도킹/언도킹 실행노드 (FSM 명령 구동, 상주)
============================================================================
patrol_commander(PC FSM)가 /robot1/dock_cmd 로 'UNDOCK'|'DOCK' 를 보내면
Pi 로컬에서 odom(+마커)으로 실행하고 /robot1/dock_done 로 결과를 발행한다.
  UNDOCK 성공 → 'UNDOCK_DONE'
  DOCK   성공 → 'DOCK_DONE'
  DOCK   실패 → 'DOCK_FAIL' (마커 못찾음/정렬실패 — FSM은 이걸 무시하고 dock_timeout→STUCK)

왜 Pi 로컬? PC→WiFi cmd_vel은 '정지'가 지연돼 뒷벽 오버슛(과거 사고). 정지 포함 모든 모션은 Pi 로컬.

동작:
  UNDOCK : odom 기준 전진 undock_distance(m) 직진(도킹존 빨강칸 탈출) → UNDOCK_DONE
  DOCK   : 마커 SEARCH(회전탐색) → ALIGN(자세정렬) → REVERSE(odom 후진) → DOCK_DONE
           (pi_dock.py 검증 로직 그대로. DOCK 전 PC 검출노드 ON 필요 → /detected_dock_pose 발행 중이어야 함)
안전: 신선 스탬프 / 전진·후진 하드캡 / 단계 타임아웃 / 완료 후 0속도 반복 /
      ★IDLE에선 cmd_vel 절대 미발행(순찰 중 Nav2 무간섭) / 진행중 새 명령 무시.

실행(Pi, 도메인 필수):
  ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 ~/pi_dock_executor.py
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# 광학→base 리맵 (x_base=z_opt 전방, y_base=-x_opt 좌+) — pi_dock.py와 동일
R_BASE_OPT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def quat_to_rotmat(x, y, z, w):
    """단위쿼터니언 → 3x3 회전행렬."""
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

IDLE, UNDOCK_FWD, SEARCH, ALIGN, CENTER, REVERSE = 'IDLE', 'UNDOCK_FWD', 'SEARCH', 'ALIGN', 'CENTER', 'REVERSE'
# ★전진주차 방식(마커=충전기쪽): 전진접근 → 제자리180° → 짧은 직진후진
FWD_APPROACH, ROTATE_180, BACK_IN = 'FWD_APPROACH', 'ROTATE_180', 'BACK_IN'
# ★3D 단계서보 정렬(아이디어C): 마커법선 기준 중심선 위로 단계이동 후 전진주차
SRV_TURN, SRV_DRIVE, SRV_FACE = 'SRV_TURN', 'SRV_DRIVE', 'SRV_FACE'
# ★v4: 도킹 직후 자세 미세보정(제자리 회전, 마커 불필요 — FACE잔차+회전부족분을 odom으로 상쇄)
POST_TRIM = 'POST_TRIM'
SRV_RANGE = 'SRV_RANGE'   # ★v4.1: FWD 직전 마커거리 표준화(깊이 일정화)


class PiDockExecutor(Node):
    def __init__(self):
        super().__init__('pi_dock_executor')
        # --- 인터페이스 토픽 (patrol_commander 네임스페이스 /robot1) ---
        self.declare_parameter('cmd_topic', '/robot1/dock_cmd')
        self.declare_parameter('done_topic', '/robot1/dock_done')
        # --- UNDOCK(전진 탈출) ---
        self.declare_parameter('undock_distance', 0.35)    # 전진 거리(m) — 도킹존 빨강칸 탈출
        self.declare_parameter('undock_speed', 0.08)       # 전진 속도(m/s)
        self.declare_parameter('undock_hard_margin', 0.10) # 하드캡 여유(m)
        self.declare_parameter('undock_timeout', 15.0)     # 초
        # --- REVERSE(후진, pi_dock/pi_reverse 검증값 동일) ---
        self.declare_parameter('distance', 0.40)   # 첫 도킹 검증값(2.5cm/2cm 똑바름). 밀수록 뒷벽 위험이라 복귀
        self.declare_parameter('speed', 0.07)
        self.declare_parameter('slowdown', 0.08)
        self.declare_parameter('min_speed', 0.045)
        self.declare_parameter('hard_margin', 0.10)   # ★odom 안전캡 여유↑ (서보 곡선후진 시 마커거리 정지가 먼저 걸리게). 마커거리 stop이 주, odom은 백업.
        self.declare_parameter('max_time', 25.0)
        # --- ★마커거리 기준 후진(시작점 변동 흡수, 벽충돌 방지) ---
        self.declare_parameter('reverse_by_range', True)   # True=마커거리로 정지(권장), False=odom거리
        self.declare_parameter('target_range', 0.680)      # ★도킹완료 마커거리(m) 튜닝값(0.692캘리브→0.680)
        # --- 마커 조향(후진 중) ---
        self.declare_parameter('use_marker_steer', True)
        self.declare_parameter('k_bearing', 0.4)   # ★튜닝: 후진 heading 항(gentle).
        self.declare_parameter('reverse_steer_sign', 1.0)
        self.declare_parameter('w_max', 0.25)
        # ★가로 서보: 마커 방향으로 로봇의 중심선 이탈(가로오프셋 m)을 계산해 0으로 몰아감.
        #   bearing만으론 못잡던 옆오차 근본보정. 노이즈 있어 gentle + 필터.
        self.declare_parameter('use_lateral_servo', True)
        self.declare_parameter('k_lateral', 2.0)      # ★튜닝: 가로오프셋(m)→조향. sign -1과 함께 수렴검증됨.
        self.declare_parameter('lateral_sign', -1.0)  # ★튜닝: -1=수렴방향(실검증). +1은 발산.
        self.declare_parameter('lateral_target', 0.075) # ★7/6 실측확정: 순찰 접근기하 보정 +0.075(수동 언도크→도크 기준은 0.06=2cm였음). 0=중심
        self.declare_parameter('marker_timeout', 1.5)    # ★튜닝: 1.0→1.5 (가장자리 순간드롭 견딤)
        self.declare_parameter('lpf_alpha', 0.5)         # ★튜닝: 0.3→0.5 (지연↓, 오버슛↓)
        # --- SEARCH(마커 탐색 회전) ---
        self.declare_parameter('search_speed', 0.12)     # ★튜닝: 0.20→0.12 (느리게, 발견시 오버슛↓)
        self.declare_parameter('search_dir', 1.0)
        self.declare_parameter('search_timeout', 70.0)   # ★7/9: 40→70 — search_speed 0.12rad/s면 1바퀴=52s인데 40s는 마커가 반대편이면 도달 전 타임아웃(실주행 2연속 40s 실패)
        self.declare_parameter('search_confirm', 3)
        # --- ALIGN(마커 정렬) ---
        self.declare_parameter('align_k', 0.6)           # ★튜닝: 0.9→0.6 (부드럽게)
        self.declare_parameter('align_w_max', 0.12)      # ★튜닝: 0.22→0.12 (오버슛→마커소실 방지)
        self.declare_parameter('align_w_min', 0.12)   # ★7/6 확정: 0.05는 정지마찰 못 이겨 ALIGN 타임아웃
        self.declare_parameter('align_tol_deg', 3.5)   # 첫 도킹 검증값 복귀
        # ★IMG_CENTER 조준(7/9 사용자 확정): '정면'의 정의 = 마커가 화면 정중앙(u=320px).
        #   bearing=0 은 광학축(u=cx=362.3px) 기준이라 오른쪽으로 42px 치우친다.
        #   target = atan((320-cx)/fx) 부호반영 = +1.911°.  캘리브 바뀌면 이 값도 갱신할 것.
        #   (fusion 의 bearing_offset_deg 1.91 과 같은 취지 — 도킹 쪽만 빠져 있었음)
        self.declare_parameter('bearing_target_deg', 1.911)
        self.declare_parameter('align_settle', 8)
        self.declare_parameter('align_timeout', 20.0)
        # --- ★CENTER(중심선 정렬: 후진 전에 lat_off=0으로 호 그려 올라타기) ---
        self.declare_parameter('use_center_align', True)   # 게이트: False면 옛동작(ALIGN→바로REVERSE)
        self.declare_parameter('center_lat_tol', 0.01)     # ★7/6 확정: 0.02는 슬랙 커서 lateral_target 덜 감
        self.declare_parameter('center_k_bear', 0.6)       # 정면유지 항(bearing→0)
        self.declare_parameter('center_k_lat', 1.5)        # 중심선보정 항(lat_off→0). 튜닝대상
        self.declare_parameter('center_lat_sign', 1.0)     # ★전진기준 부호(실로봇서 반대면 -1). 후진(-1)과 다름
        self.declare_parameter('center_w_max', 0.15)       # 각속도 상한
        self.declare_parameter('center_speed', 0.05)       # 전진속도(느리게, 벽접근 주의)
        self.declare_parameter('center_range_floor', 0.25) # 이보다 마커 가까우면 전진금지(벽충돌 방지)
        self.declare_parameter('center_settle', 6)         # 양축 만족 연속프레임
        self.declare_parameter('center_timeout', 15.0)     # 초과시 정렬포기 → REVERSE(옛 서보 폴백)
        # --- ★전진주차 방식 (dock_mode='park'): 마커=충전기쪽. 전진접근→180°→짧은후진 ---
        self.declare_parameter('dock_mode', 'park')        # 'park'=신규(전진주차+180°+후진), 'reverse'=옛 후진도킹
        # ★전진 정지 기준: 'odom'=실주행거리(자로 실측한 값) / 'range'=마커거리
        self.declare_parameter('fwd_mode', 'odom')         # 사용자 실측 기준 = odom
        self.declare_parameter('fwd_dist', 0.30)           # ★odom 전진거리(m) — 실측 30cm
        # FWD_APPROACH(range모드): 마커를 정면으로 전진, 정확한 거리서 정지
        self.declare_parameter('fwd_park_range', 0.35)     # ★전진 정지 마커거리(m). 카메라 선명한 거리
        self.declare_parameter('fwd_speed', 0.06)          # 전진속도
        self.declare_parameter('fwd_min_speed', 0.03)
        self.declare_parameter('fwd_slowdown', 0.15)       # 목표 이 거리전부터 감속(m)
        self.declare_parameter('fwd_k_bearing', 0.8)       # 전진중 중심유지 조향
        self.declare_parameter('fwd_w_max', 0.15)
        self.declare_parameter('fwd_timeout', 25.0)
        self.declare_parameter('fwd_range_floor', 0.20)    # 이보다 가까우면 정지(벽/마커 충돌방지)
        # ROTATE_180: 제자리 회전(odom yaw 누적)
        self.declare_parameter('rotate_deg', 180.0)        # 회전각(도). 접점 정렬 위해 175~185 미세조정 가능
        self.declare_parameter('rotate_speed', 0.4)        # 회전 각속도(rad/s)
        self.declare_parameter('rotate_dir', 1.0)          # +1=좌회전, -1=우회전
        self.declare_parameter('rotate_tol_deg', 1.0)
        self.declare_parameter('rotate_timeout', 20.0)
        # BACK_IN: 짧은 직진 후진(odom거리)으로 충전기 접점 연결
        self.declare_parameter('back_in_dist', 0.20)       # ★후진 거리(m). 7/4 최종: 0.20이 최고성적(4cm 얕음). 라이다 정지 켜면 하드캡 역할
        self.declare_parameter('back_in_speed', 0.05)
        self.declare_parameter('back_in_timeout', 15.0)
        # --- ★3D 단계서보 정렬(아이디어C, 2026-07-04): SEARCH→(서보)→FWD_APPROACH ---
        #   마커 full 6DOF 법선으로 '충전기 중심선 위 servo_standoff 정면'에 단계이동.
        #   기존 ALIGN(bearing만)의 옆이탈 문제 보정. 단계분리로 서보발산 방지.
        #   False면 옛동작(SEARCH→ALIGN). 🔴실주행 미검증(시뮬 수렴만 확인).
        # ★7/9 기본 OFF로 강등: 접근점 P = 마커 + standoff·법선 이 '법선'에 의존하는데,
        #   평면마커 자세추정(IPPE_SQUARE)은 두 해 모호성이 있고 화각이 좁을수록(우리 28°)
        #   심해진다. 실주행에서 마커가 정면(bear+2°)인데 P가 왼쪽 90°(hdg+84°)로 찍혀
        #   그쪽으로 돌다 마커를 화각 밖으로 놓치고 SEARCH 무한루프에 빠졌다.
        #   근사K 시절엔 법선이 뭉개져 늘 '접근점 위' 판정→FACE 직행이라 문제가 숨어 있었음.
        #   ★해결(7/9): 검출기(aruco_dock_detector)가 solvePnPGeneric 으로 두 해의 재투영오차를
        #   비교해, 모호하면(=마커가 정면) 법선을 시선방향으로 대체한다. 90° 뒤집힘이 사라져
        #   법선을 다시 신뢰할 수 있으므로 기본 True 복귀. 문제 재발 시 False로 우회(ALIGN 경로).
        self.declare_parameter('use_servo_align', True)
        self.declare_parameter('servo_standoff', 0.734)  # ★7/9 캘리브 실측: 근사K(fx554) 시절 0.33 = 실제 0.734m. 실측K(fx1268.3) 적용 후 measure_dock_range.py 로 확정(σ0.3mm)
        self.declare_parameter('srv_pos_tol', 0.067)     # ★7/9 재스케일(×2.224): 접근점 도달 판정(m)
        self.declare_parameter('srv_turn_tol_deg', 3.0)  # 접근점 조준 완료각
        self.declare_parameter('srv_ok_count', 3)        # 연속 N프레임 만족 시 전환(노이즈 방지)
        self.declare_parameter('srv_v_max', 0.06)        # 접근 전진속도 상한
        self.declare_parameter('srv_k_v', 0.8)
        self.declare_parameter('srv_turn_timeout', 12.0)
        self.declare_parameter('srv_drive_timeout', 25.0)
        self.declare_parameter('srv_face_timeout', 12.0)
        self.declare_parameter('srv_normal_min', 0.5)    # 법선 XY성분 미달(마커가 바닥/천장 봄) → ALIGN 폴백
        # ★WALL_NORMAL(7/9): 마커 법선을 카메라가 아니라 '전방 벽 라이다 피팅'에서 얻는다.
        #   왜: 정면 평면마커는 IPPE 두 해가 시선축 거울상이라 좌우 기울기 부호를 원리적으로 못 가린다.
        #   검출기의 frontal fallback(법선=시선방향)은 안전하지만 'P=현재위치'가 되어 SRV_DRIVE(횡보정)를
        #   통째로 건너뛴다 → 중심선에서 벗어난 채 출발하면 그 오차가 도킹존까지 실려감(실주행 좌 10cm).
        #   마커는 벽에 평평히 붙어 있으므로 마커 법선 = 벽 법선. 벽은 라이다가 모호성 없이 직접 본다.
        #   (POST_TRIM이 뒷벽으로 하는 것과 같은 계산을 전방에 적용)
        # ★7/9 실주행 실패로 기본 OFF 강등: 켜니 SRV_FACE가 정렬을 못 끝내고 12s 타임아웃 → DOCK_FAIL ×2
        #   (로봇이 프리도킹 앞에서 좌우로만 흔들림). 전방 ±30° 창에 벽이 아니라 충전기 구조물/옆 로봇이
        #   잡혀 피팅 기울기가 왜곡된 것으로 추정. 창/검증 조건을 잡기 전엔 켜지 말 것.
        #   OFF면 _servo_normal 이 카메라 법선(정면 폴백)을 반환 → 7/9 성공 구성과 동일.
        self.declare_parameter('use_wall_normal', True)
        # ★7/9 실측 튜닝: ±30°창은 0.41m 물체(충전기 구조물)가 섞여 기울기 -23.7°/잔차 91mm(평면 아님).
        #   ±10°로 좁히면 기울기 -1.5°/잔차 14.6mm = 진짜 벽. 창 축소 + 거리밴드 + 잔차검증 3중 방어.
        self.declare_parameter('wall_normal_window_deg', 24.0)  # 전방 피팅 창(±12°)
        self.declare_parameter('wall_normal_min_points', 12)
        self.declare_parameter('wall_normal_max_deg', 25.0)     # 벽이 이보다 더 비스듬하면 무효(→카메라 법선 폴백)
        self.declare_parameter('wall_range_band', 0.15)         # 중앙값 거리에서 ±이 범위 밖 점은 벽이 아님(이상치 제거)
        # ★7/9: 0.030은 실측 잔차(22~32mm)와 겹쳐 매 틱 유효/무효가 깜빡였다. 여유를 둔다.
        self.declare_parameter('wall_max_resid', 0.045)         # 피팅 잔차 중앙값 상한(m). 넘으면 평면 아님 → 폴백
        self.declare_parameter('wall_slope_median_n', 5)
        # ★LIDAR_YAW_OFF(7/14, 로봇2): 라이다 장착각 보정[deg]. 벽피팅이 '라이다 기준 평행'이라
        #   라이다가 몸체 대비 돌아가 있으면 몸이 비틀린 채 정렬됨 — 로봇별 값(로봇1=0).
        self.declare_parameter('wall_yaw_offset_deg', 0.0)         # 벽 기울기 시간 중앙값 창(단발 피팅 ±5° 흔들림 억제)
        # ★v3(7/4 사용자 아이디어): FACE 목표=마커 '법선각'(=마커 yaw 반대). bearing(중심응시)은
        #   옆이탈 시 비틀림 유발 → 법선 정렬이면 옆에 있어도 자세 똑바름.
        self.declare_parameter('srv_face_normal', True)     # False=옛동작(bearing→0)
        # ★7/9: 0.044 → 0.30 으로 크게 올려 SRV_TURN/SRV_DRIVE(게걸음)를 사실상 봉인.
        #   이유: so=min(standoff,d) 라 로봇이 마커와 같은 거리에서 옆으로 조금만 벗어나도
        #   접근점 P가 '로봇 바로 옆'(hdg≈±90°)에 찍힌다. 화각 28°에선 그쪽으로 도는 순간
        #   마커가 프레임을 벗어나 SEARCH 무한루프(실주행 hdg+84° 사례). 좁은 화각 ≠ 게걸음.
        #   → 횡오차는 FWD_APPROACH 중 곡선 조향(use_lateral_fwd)으로 흡수한다.
        self.declare_parameter('srv_face_direct', 0.30)     # FACE 직행 가드(m)
        # ★LATERAL_FWD(7/9): 전진 접근 중 '조준오차 + 횡오차' 복합 조향으로 중심선에 올라탄다.
        #   횡오차는 라이다 벽법선 기준(_lat_off_wall)이라 카메라 모호성 무관. 무효면 조준만 사용.
        self.declare_parameter('use_lateral_fwd', True)
        self.declare_parameter('k_lat_fwd', 1.2)            # 횡오차(m) → 각속도. 튜닝 대상
        self.declare_parameter('lat_fwd_sign', 1.0)         # 실기서 발산하면 -1
        self.declare_parameter('lat_fwd_w_max', 0.15)       # 복합 조향 각속도 상한(rad/s)
        self.declare_parameter('fwd_straight_after_servo', True)  # 서보정렬 후 FWD는 직진(조향OFF, 비틀림 재생성 방지)
        # --- ★v4(7/4 사용자 요청): 도킹후 자세보정 + 후진 직진성 + 옆쏠림 계통보정 ---
        self.declare_parameter('use_post_trim', True)       # BACK_IN 후 제자리 yaw 미세보정(마커 불필요, odom 기준)
        # ★7/9 P제어 전환: 뱅뱅(copysign 고정속도)은 tol 0.3°를 한 틱에 넘겨 절대 수렴 못하고
        #   ±2° 진동 후 타임아웃으로 끝났다(실주행 로그 확인). 오차비례 + 정지마찰 최소값으로 교체.
        self.declare_parameter('post_trim_tol_deg', 0.8)   # 벽피팅 노이즈(±1~2°)보다 크게 — 타임아웃 대신 정상수렴
        self.declare_parameter('post_trim_speed', 0.15)     # 각속도 상한(rad/s)
        self.declare_parameter('post_trim_k', 1.2)          # P 게인 (w = k·yerr)
        self.declare_parameter('post_trim_w_min', 0.05)     # 정지마찰 돌파 최소 각속도(rad/s)
        self.declare_parameter('trim_median_n', 5)          # 벽각 중앙값 필터 창(회전 중 스캔왜곡 억제)
        self.declare_parameter('post_trim_timeout', 8.0)
        self.declare_parameter('back_heading_hold', True)   # BACK_IN 중 odom yaw 홀드(비틀림 누적 방지)
        self.declare_parameter('back_k_yaw', 1.5)
        self.declare_parameter('back_w_max', 0.3)
        # 옆쏠림 계통편향 보정(m, +=왼쪽으로 중심선 이동). 오른쪽벽에 붙으면 +0.03~0.04 실험
        self.declare_parameter('srv_lat_offset', 0.0)
        # ★v4.1 SRV_RANGE: FWD 직전 마커거리를 servo_standoff로 표준화 → 도킹 깊이 런간 일정
        #   (odom 고정 30cm 전진은 '시작거리 편차'를 그대로 최종깊이로 전달: +10/-5cm 널뛰던 원인)
        self.declare_parameter('srv_range_enable', True)
        self.declare_parameter('srv_range_tol', 0.033)     # ★7/9 재스케일(×2.224). 거리 허용오차(마커단위 m)
        self.declare_parameter('srv_range_speed', 0.035)   # 미세 전/후진 속도
        self.declare_parameter('srv_range_timeout', 12.0)
        # --- ★v5(7/4 밤 설계, 🔴미검증): BACK_IN 라이다 정지 — 후방 벽거리 절대기준 ---
        #   깊이 ±5cm 런간 편차의 근본해법. odom/마커거리(상대값) 대신 '벽까지 실거리'로 정지.
        #   ⚠사용법: 다음 세션에 캘리브 필수 — 로봇을 완벽 도킹위치에 손으로 놓고
        #     후방거리 읽기(로그 'BACK_LIDAR rear=' 또는 /pi_dock_executor 파라미터 체크 스크립트)
        #     → back_wall_target 에 그 값 설정 → use_back_lidar true.
        self.declare_parameter('use_back_lidar', True)      # ★7/4 캘리브+실주행 검증(0.190 재현 2mm) → 기본ON 구움
        self.declare_parameter('back_wall_target', 0.190)   # ★7/4 자(ruler) 확정: 앞바퀴~벽 24.5cm = 라이다 0.190 (눈짐작 -5cm 시도는 과깊음으로 롤백)
        self.declare_parameter('back_wall_window_deg', 20.0)# 후방 스캔 창(±10°)
        self.declare_parameter('back_wall_min_points', 5)   # 유효포인트 최소개수(미달=라이다 무효→odom 폴백)
        self.declare_parameter('back_hard_extra', 0.12)     # odom 하드캡 여유(0.06→0.12: target 5cm 깊어져 후진 ~0.25m 필요)
        # --- ★v5.1 WALL_TRIM(7/4 사용자 아이디어): POST_TRIM을 라이다 벽각도(절대기준)로 ---
        #   odom yaw_ref(상대)는 FACE잔차·드리프트만큼 절대 틀어짐 잔존. 후진 후 뒷벽 점들을
        #   직선피팅하면 벽 대비 절대각이 나옴 → 그만큼 회전=벽과 완전 평행. 무효시 odom 폴백.
        self.declare_parameter('use_wall_trim', True)
        self.declare_parameter('wall_trim_window_deg', 60.0)  # 피팅 창(±30°, 넓을수록 각도분해능↑)
        self.declare_parameter('wall_trim_min_points', 15)
        self.declare_parameter('wall_trim_max_deg', 15.0)     # ★클램프 상한(거부X). 8° 거부는 큰 틀어짐일수록 폴백하는 역설계라 폐기
        self.declare_parameter('trim_settle_ticks', 6)        # ★정지 후 연속 N틱 tol 안이어야 완료(회전 중 스캔왜곡 방지)

        g = lambda n: self.get_parameter(n).value
        self.distance = float(g('distance')); self.speed = float(g('speed'))
        self.slowdown = float(g('slowdown')); self.min_speed = float(g('min_speed'))
        self.hard_margin = float(g('hard_margin')); self.max_time = float(g('max_time'))

        self.phase = IDLE
        self.odom_xy = None
        self.start_xy = None
        self.phase_t0 = None
        self.bearing_f = None
        self.lat_off_f = None      # 가로오프셋(m) 필터값(중심선 이탈)
        self.marker_range_f = None # 마커까지 거리(m) 필터값
        self.last_marker = None
        self.marker_hits = 0
        self.align_ok = 0
        self.center_ok = 0
        self.odom_yaw = None        # ★odom yaw(rad)
        self.rot_accum = 0.0        # 180°회전 누적각
        # ★서보 상태
        self.mx_f = None; self.my_f = None   # 마커 base 위치(LPF)
        self.nx_f = None; self.ny_f = None   # 마커 법선(base, 마커→로봇쪽 플립, LPF)
        self.srv_ok = 0
        self.via_servo = False               # ★서보로 정렬됐나(FWD 직진 판단용)
        self.yaw_ref = None                  # ★v4: FACE완료 시 '완벽정렬 odom yaw'(잔차 제거값)
        self.back_yaw0 = None                # ★v4: BACK_IN 시작 yaw(직진홀드)
        self.scan_msg = None                 # ★v5: 최신 스캔(후방 벽거리용)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.rot_last_yaw = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_marker, 10)
        self.create_subscription(String, g('cmd_topic'), self.on_cmd, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(String, g('done_topic'), 10)
        self.create_timer(0.02, self.loop)   # 50Hz
        self.get_logger().info(
            f"pi_dock_executor 상주 시작: '{g('cmd_topic')}' 대기 (UNDOCK/DOCK). "
            f"IDLE에선 cmd_vel 미발행(Nav2 무간섭).")

    # ================= 콜백 =================
    def on_odom(self, msg):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation   # ★odom yaw (180° 회전용)
        self.odom_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def on_marker(self, msg):
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        p = R_BASE_OPT @ tvec               # 마커 위치(base)
        x, y = float(p[0]), float(p[1])
        bearing = math.atan2(y, x)          # 좌+
        # ★가로오프셋: 마커 방향까지 써서 로봇(base원점)이 마커 중심선서 얼마나 벗어났나.
        #   로봇을 마커프레임으로 변환 → 마커프레임 x성분(=좌우) = 중심선 이탈 거리(부호).
        q = msg.pose.orientation
        R_bm = R_BASE_OPT @ quat_to_rotmat(q.x, q.y, q.z, q.w)   # 마커 방향(base)
        p_robot_in_marker = R_bm.T @ (-p)   # 로봇 위치를 마커프레임에서 본 것
        lat_off = float(p_robot_in_marker[0])
        rng = float(np.linalg.norm(tvec))   # 마커까지 거리
        a = self.get_parameter('lpf_alpha').value
        self.bearing_f = bearing if self.bearing_f is None else (a * bearing + (1 - a) * self.bearing_f)
        self.lat_off_f = lat_off if self.lat_off_f is None else (a * lat_off + (1 - a) * self.lat_off_f)
        self.marker_range_f = rng if self.marker_range_f is None else (a * rng + (1 - a) * self.marker_range_f)
        # ★서보용: 마커 base 위치 + 법선(마커 +Z=면 바깥). 법선은 '마커→로봇' 방향으로 플립 정규화.
        n3 = R_bm @ np.array([0.0, 0.0, 1.0])
        nx, ny = float(n3[0]), float(n3[1])
        nn = math.hypot(nx, ny)
        if nn > 1e-6:
            nx, ny = nx / nn, ny / nn
            if nx * (-x) + ny * (-y) < 0.0:   # 로봇 반대편 향하면 플립
                nx, ny = -nx, -ny
            self.nx_f = nx if self.nx_f is None else (a * nx + (1 - a) * self.nx_f)
            self.ny_f = ny if self.ny_f is None else (a * ny + (1 - a) * self.ny_f)
        self.mx_f = x if self.mx_f is None else (a * x + (1 - a) * self.mx_f)
        self.my_f = y if self.my_f is None else (a * y + (1 - a) * self.my_f)
        self.last_marker = self.get_clock().now()

    def on_cmd(self, msg):
        c = (msg.data or '').strip().upper()
        # ★ESTOP_CANCEL(7/10): CANCEL은 phase 가드보다 먼저 처리한다.
        #   CANCEL은 '진행중'일 때만 의미가 있는데, 아래 가드가 바로 그때 모든 명령을 버렸다
        #   → PC가 EMERGENCY_STOP에 CANCEL을 보내도 Pi는 계속 굴렀다(실측: 31.8s, 3분55초).
        #   PC의 20Hz 0속도는 같은 /cmd_vel을 두고 경쟁할 뿐 — 바퀴를 멈추는 건 여기뿐이다.
        if c in ('CANCEL', 'ESTOP', 'STOP'):
            self.cancel_motion(f'PC {c} 수신')
            return
        if self.phase != IDLE:
            self.get_logger().warn(f'dock_cmd={c} 무시 — 이미 진행중({self.phase})')
            return
        if c == 'UNDOCK':
            self.start_xy = None
            self.set_phase(UNDOCK_FWD)
        elif c == 'DOCK':
            # 마커 상태 리셋 후 SEARCH부터
            self.bearing_f = None; self.lat_off_f = None; self.marker_range_f = None; self.last_marker = None
            self.mx_f = None; self.my_f = None; self.nx_f = None; self.ny_f = None; self.srv_ok = 0  # ★서보 리셋
            self.via_servo = False
            self.yaw_ref = None; self.back_yaw0 = None   # ★v4 리셋
            self.align_ok = 0; self.center_ok = 0
            self.marker_hits = 0; self.align_ok = 0; self.start_xy = None
            self.set_phase(SEARCH)
        else:
            self.get_logger().warn(f'알 수 없는 dock_cmd: {msg.data!r}')

    # ================= 유틸 =================
    def marker_fresh(self, now):
        return (self.last_marker is not None and self.bearing_f is not None and
                (now - self.last_marker).nanoseconds / 1e9 <= self.get_parameter('marker_timeout').value)

    @staticmethod
    def _norm_ang(a):
        """각도 정규화(-pi~pi)."""
        return math.atan2(math.sin(a), math.cos(a))

    def _aim(self, ang):
        """★IMG_CENTER: '정면' 기준을 화면 정중앙으로 옮긴 조준오차.
        ang(=bearing 또는 법선각)에서 bearing_target_deg 를 빼서, 0이면 마커가 화면 한가운데."""
        if ang is None:
            return 0.0
        return self._norm_ang(ang - math.radians(self.get_parameter('bearing_target_deg').value))

    def _on_scan(self, msg):
        self.scan_msg = msg

    def _rear_wall_dist(self):
        """★v5: 후방(스캔각 180°±창/2) 유효거리 중앙값. 무효면 None(→odom 폴백)."""
        m = self.scan_msg
        if m is None or not m.ranges:
            return None
        half = math.radians(self.get_parameter('back_wall_window_deg').value) / 2.0
        vals = []
        a = m.angle_min
        for r in m.ranges:
            if abs(self._norm_ang(a - math.pi)) <= half:
                if math.isfinite(r) and m.range_min < r < m.range_max:
                    vals.append(r)
            a += m.angle_increment
        if len(vals) < int(self.get_parameter('back_wall_min_points').value):
            return None
        vals.sort()
        return vals[len(vals)//2]

    def _rear_wall_angle(self):
        """★v5.1: 후방벽 직선피팅 → 로봇의 수직정렬 대비 CCW 틀어짐각 φ(rad).
        base 좌표 x=r·cos(a), y=r·sin(a)로 벽점들을 x=c+m·y 회귀 → φ=atan(m).
        (완벽 수직이면 벽이 y축 평행=m0. 로봇이 +ε CCW 돌면 m=tanε. 7/4 실측 프로파일로 부호검증.)
        무효(포인트 부족/창 좁음/이상치)면 None → 호출측 odom 폴백."""
        m = self.scan_msg
        if m is None or not m.ranges:
            return None
        half = math.radians(self.get_parameter('wall_trim_window_deg').value) / 2.0
        xs, ys = [], []
        a = m.angle_min
        for r in m.ranges:
            if abs(self._norm_ang(a - math.pi)) <= half and \
               math.isfinite(r) and m.range_min < r < m.range_max:
                xs.append(r * math.cos(a)); ys.append(r * math.sin(a))
            a += m.angle_increment
        n = len(xs)
        if n < int(self.get_parameter('wall_trim_min_points').value):
            self.get_logger().warn(f'wall_fit 무효: 포인트 {n}개 부족', throttle_duration_sec=1.0)
            return None
        my_ = sum(ys) / n; mx_ = sum(xs) / n
        syy = sum((y - my_) ** 2 for y in ys)
        if syy < 1e-4:   # y 스프레드 너무 좁음(각도 분해능 없음)
            self.get_logger().warn(f'wall_fit 무효: y스프레드 부족(syy={syy:.2e})', throttle_duration_sec=1.0)
            return None
        sxy = sum((xs[i] - mx_) * (ys[i] - my_) for i in range(n))
        phi = math.atan(sxy / syy)
        phi -= math.radians(float(self.get_parameter('wall_yaw_offset_deg').value))   # ★LIDAR_YAW_OFF
        cap = math.radians(self.get_parameter('wall_trim_max_deg').value)
        phi = max(-cap, min(cap, phi))   # ★거부 대신 클램프(닫힌루프가 매 틱 재측정하므로 안전)
        # ★7/9 중앙값 필터: 단발 피팅은 회전 중 스캔왜곡/이상치로 ±2° 튄다. 최근 N개 중앙값 반환.
        n = max(1, int(self.get_parameter('trim_median_n').value))
        hist = getattr(self, '_wall_hist', None)
        if hist is None:
            hist = self._wall_hist = []
        hist.append(phi)
        if len(hist) > n:
            hist.pop(0)
        s = sorted(hist)
        return s[len(s) // 2]

    def _front_wall_normal(self):
        """★WALL_NORMAL: 전방 벽 직선피팅 → '벽에서 로봇으로' 향하는 단위법선 (base 좌표).
        마커가 붙은 벽의 법선 = 마커 법선. 라이다라 좌우 모호성이 없다.
        무효(포인트 부족/스프레드 부족/벽이 과도하게 비스듬)면 None → 카메라 법선 폴백."""
        m = self.scan_msg
        if m is None or not m.ranges:
            return None
        half = math.radians(self.get_parameter('wall_normal_window_deg').value) / 2.0
        pts = []
        a = m.angle_min
        for r in m.ranges:
            if abs(self._norm_ang(a)) <= half and \
               math.isfinite(r) and m.range_min < r < m.range_max:
                pts.append((r * math.cos(a), r * math.sin(a), r))
            a += m.angle_increment
        if len(pts) < 5:
            return None
        # ①거리밴드: 벽면은 거의 같은 거리대. 중앙값에서 크게 벗어난 점(앞에 낀 구조물/로봇) 제거.
        rs = sorted(p[2] for p in pts)
        rmed = rs[len(rs) // 2]
        band = self.get_parameter('wall_range_band').value
        pts = [p for p in pts if abs(p[2] - rmed) <= band]
        n = len(pts)
        if n < int(self.get_parameter('wall_normal_min_points').value):
            return None
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        mx_ = sum(xs) / n; my_ = sum(ys) / n
        syy = sum((y - my_) ** 2 for y in ys)
        if syy < 1e-4:
            return None
        sxy = sum((xs[i] - mx_) * (ys[i] - my_) for i in range(n))
        slope = sxy / syy                      # 벽: x = c + slope*y
        # ②잔차검증: 진짜 평면이면 잔차가 작다(실측 벽 14.6mm vs 오염 91.4mm).
        res = sorted(abs(xs[i] - (mx_ + slope * (ys[i] - my_))) for i in range(n))
        if res[len(res) // 2] > self.get_parameter('wall_max_resid').value:
            self.get_logger().warn(
                f'벽 피팅 기각: 잔차 {res[len(res)//2]*1000:.0f}mm — 평면 아님(→카메라 법선)',
                throttle_duration_sec=2.0)
            return None
        # ③기울기 상한
        if abs(math.degrees(math.atan(slope))) > self.get_parameter('wall_normal_max_deg').value:
            return None                        # 벽이 너무 비스듬 = 벽이 아닐 가능성
        # ④시간 중앙값: 단발 피팅은 창/노이즈에 따라 ±5°씩 흔들린다(실측 -6.5° vs +4.2°).
        #   그대로 쓰면 SRV_FACE가 흔들리는 목표를 쫓아 수렴하지 못한다.
        k = max(1, int(self.get_parameter('wall_slope_median_n').value))
        hist = getattr(self, '_fslope_hist', None)
        if hist is None:
            hist = self._fslope_hist = []
        hist.append(slope)
        if len(hist) > k:
            hist.pop(0)
        if len(hist) < k:
            return None                        # 아직 표본 부족 → 폴백(카메라 법선)
        s = sorted(hist)
        slope = s[len(s) // 2]
        # 직선 x=c+slope*y 의 법선 ∝ (1,-slope). 로봇(원점)을 향하도록 부호 반전.
        nx, ny = -1.0, slope
        h = math.hypot(nx, ny)
        return (nx / h, ny / h)

    def _lat_off_wall(self):
        """★WALL_NORMAL: 로봇이 '마커 중심선'에서 좌우로 벗어난 거리(m, 부호 있음).
        중심선 = 마커를 지나 벽 법선 방향으로 뻗은 직선. 접선 t = n을 90° CCW 회전.
        lat = (마커→로봇) · t.  법선을 라이다에서 얻으므로 IPPE 모호성과 무관.
        ※용도 분리(7/9): 벽 법선은 '횡오차 계산 전용'. 각도 정렬(SRV_FACE)에는 절대 쓰지 않는다 —
          '마커를 본다'와 '벽과 직각'은 다른 목표라, 한 제어 루프에서 두 기준을 섞으면
          매 틱 부호가 뒤집혀 좌우로 흔들리기만 한다(실주행 wall+14.5°/cam-14.0° 교대 → 타임아웃).
        무효면 None → 호출측은 조준(bearing)만 사용."""
        if self.get_parameter('use_wall_normal').value is not True or self.mx_f is None:
            return None
        wn = self._front_wall_normal()
        if wn is None:
            return None
        nx, ny = wn
        if (nx * (-self.mx_f) + ny * (-self.my_f)) < 0.0:   # 마커→로봇 방향으로 정렬
            nx, ny = -nx, -ny
        tx, ty = -ny, nx                     # 법선의 좌수직(접선)
        return (-self.mx_f) * tx + (-self.my_f) * ty

    def _servo_normal(self):
        """서보 각도정렬(SRV_FACE)·접근점(_srv_target)이 쓸 법선 = 카메라 법선 전용.
        검출기가 모호할 땐 시선방향으로 대체해 주므로 안전하고, 마커를 계속 화각에 잡아둔다.
        (벽 법선은 _lat_off_wall 의 횡오차 계산에만 쓴다 — 위 주석 참조)"""
        if self.nx_f is None:
            return None
        return (self.nx_f, self.ny_f, 'cam')

    def _srv_target(self):
        """★서보 접근점 P = 마커중심 + 법선*standoff (base 상대좌표).
        법선 퇴화(마커가 위/아래 봄) 시 None → 호출측이 ALIGN 폴백."""
        if self.mx_f is None:
            return None
        nrm = self._servo_normal()
        if nrm is None or math.hypot(nrm[0], nrm[1]) < self.get_parameter('srv_normal_min').value:
            return None
        nx_use, ny_use = nrm[0], nrm[1]
        # ★v2(7/4 실주행 발산픽스): pre-dock서 로봇이 이미 standoff 거리라 P가 발밑/뒤로
        #   떨어져 hdg ±180° 널뛰기 발산. → standoff_eff=max(standoff, 현재 마커거리):
        #   가까울 땐 '같은 거리의 중심선 위 지점'으로 옆 호 이동.
        so = self.get_parameter('servo_standoff').value
        d = math.hypot(self.mx_f, self.my_f)
        so = min(so, d)   # ★min: 안쪽이면 현재거리(옆 호), 밖이면 standoff(접근)
        px = self.mx_f + so * nx_use
        py = self.my_f + so * ny_use
        # ★v4: 옆쏠림 계통편향 보정 — 접근방향(-n)의 왼쪽 수직으로 lat_offset 이동
        lat = self.get_parameter('srv_lat_offset').value
        if lat != 0.0:
            px += lat * ny_use
            py += lat * (-nx_use)
        return (px, py)

    def send(self, v, w=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v); cmd.twist.angular.z = float(w)
        self.pub.publish(cmd)

    def set_phase(self, ph):
        self.phase = ph; self.phase_t0 = self.get_clock().now()
        self.get_logger().info(f'--- 단계: {ph} ---')

    def _to_idle(self):
        self.phase = IDLE; self.phase_t0 = None

    def finish(self, done_msg):
        """성공 완료: 0속도 확실히 → done 발행 → IDLE."""
        for _ in range(5):
            self.send(0.0)
        self.done_pub.publish(String(data=done_msg))
        self.get_logger().info(f'{done_msg} 발행 → IDLE')
        self._to_idle()

    def cancel_motion(self, reason):
        """★ESTOP_CANCEL(7/10): 비상정지/취소 — 0속도 확실히 → IDLE. done은 발행하지 않는다.
        (DOCK_FAIL을 쏘면 PC가 '도킹 실패'로 오해해 STUCK 재시도를 돌린다. 취소는 실패가 아니다.
         PC는 EMERGENCY_STOP 해제 후 RESUME 시 DOCK를 다시 발행한다.)
        IDLE이어도 조용히 0속도를 박아둔다(멱등 — 연타/중복 CANCEL 안전)."""
        was = self.phase
        for _ in range(5):
            self.send(0.0)
        self._to_idle()
        self.get_logger().warn(f'★취소({reason}): {was} → 0속도, IDLE (done 미발행)')

    def abort_dock(self, reason):
        """DOCK 실패: 0속도 → DOCK_FAIL 발행(가짜 성공 금지) → IDLE. FSM은 dock_timeout→STUCK."""
        for _ in range(5):
            self.send(0.0)
        self.done_pub.publish(String(data='DOCK_FAIL'))
        self.get_logger().warn(f'DOCK 실패({reason}) → DOCK_FAIL 발행, IDLE (FSM 타임아웃→STUCK)')
        self._to_idle()

    # ================= 메인 루프 =================
    def loop(self):
        now = self.get_clock().now()
        if self.phase == IDLE:
            return   # ★ cmd_vel 미발행 — 순찰 중 Nav2와 절대 안 싸움
        if self.odom_xy is None:
            return
        if self.phase_t0 is None:
            self.phase_t0 = now
        el = (now - self.phase_t0).nanoseconds / 1e9

        # ---------- UNDOCK_FWD (전진 탈출) ----------
        if self.phase == UNDOCK_FWD:
            if self.start_xy is None:
                self.start_xy = self.odom_xy
                self.get_logger().info('전진 undock 시작'); return
            traveled = math.hypot(self.odom_xy[0] - self.start_xy[0], self.odom_xy[1] - self.start_xy[1])
            dist = self.get_parameter('undock_distance').value
            cap = dist + self.get_parameter('undock_hard_margin').value
            if traveled >= cap:
                self.get_logger().warn(f'undock 안전캡 급정지 {traveled*100:.1f}cm'); self.finish('UNDOCK_DONE'); return
            if el > self.get_parameter('undock_timeout').value:
                self.get_logger().warn('undock 타임아웃 급정지'); self.finish('UNDOCK_DONE'); return
            if traveled >= dist:
                self.get_logger().info(f'undock 완료: 전진 {traveled*100:.1f}cm'); self.finish('UNDOCK_DONE'); return
            self.send(self.get_parameter('undock_speed').value, 0.0)   # 직진 전진
            self.get_logger().info(f'undock 전진 {traveled*100:.1f}/{dist*100:.0f}cm', throttle_duration_sec=0.25)
            return

        # ---------- SEARCH (pi_dock 검증 로직) ----------
        if self.phase == SEARCH:
            if self.marker_fresh(now):
                self.marker_hits += 1
                if self.marker_hits >= self.get_parameter('search_confirm').value:
                    self.send(0.0)
                    if self.get_parameter('use_servo_align').value:   # ★3D 서보 게이트
                        self.get_logger().info(
                            f'마커 발견 (bear={math.degrees(self.bearing_f):+.0f}°) → SRV_TURN(3D 서보정렬)')
                        self.srv_ok = 0; self.set_phase(SRV_TURN); return
                    self.get_logger().info(f'마커 발견 (bear={math.degrees(self.bearing_f):+.0f}°) → ALIGN')
                    self.set_phase(ALIGN); self.align_ok = 0; return
            else:
                self.marker_hits = 0
            if el > self.get_parameter('search_timeout').value:
                self.abort_dock('SEARCH 타임아웃 — 마커 못찾음(검출노드 ON? 마커 가림?)'); return
            self.send(0.0, self.get_parameter('search_dir').value * self.get_parameter('search_speed').value)
            self.get_logger().info(f'SEARCH 회전중 {el:.0f}s', throttle_duration_sec=1.0)
            return

        # ---------- ★3D 단계서보 (SRV_TURN→SRV_DRIVE→SRV_FACE→FWD_APPROACH) ----------
        #   접근점 P(마커+법선*standoff)로: 조준회전 → 저속접근 → 마커 정면응시.
        #   단계분리로 발산 방지. 마커 소실=정지 후 SEARCH 복귀(ALIGN과 동일 정책).
        if self.phase in (SRV_TURN, SRV_DRIVE, SRV_FACE):
            if not self.marker_fresh(now):
                if el > 1.5:
                    self.get_logger().warn(f'{self.phase} 중 마커 소실 → SEARCH 복귀')
                    self.send(0.0); self.set_phase(SEARCH); self.marker_hits = 0; return
                self.send(0.0); return
            g = lambda n: self.get_parameter(n).value
            wmax = g('align_w_max'); kw = g('align_k'); wmin = g('align_w_min')
            tgt = self._srv_target()
            if tgt is None:
                self.get_logger().warn('서보: 법선 퇴화/미수신 → ALIGN 폴백')
                self.set_phase(ALIGN); self.align_ok = 0; return

            if self.phase == SRV_TURN:      # 접근점 조준 회전
                # ★가드: P가 사실상 현위치면 방향=노이즈 → 이동 생략, FACE 직행
                #   (v3: 8cm→srv_face_direct(기본4cm)로 조임 — 옆이탈 보정이 실제로 돌게)
                if math.hypot(tgt[0], tgt[1]) < max(g('srv_pos_tol'), g('srv_face_direct')):
                    self.get_logger().info('서보: 이미 접근점 위 → SRV_FACE 직행')
                    self.srv_ok = 0; self._fslope_hist = []   # ★벽 기울기 이력 초기화
                    self.set_phase(SRV_FACE); return
                hdg = math.atan2(tgt[1], tgt[0])
                if abs(hdg) < math.radians(g('srv_turn_tol_deg')):
                    self.srv_ok += 1; self.send(0.0)
                    if self.srv_ok >= g('srv_ok_count'):
                        self.srv_ok = 0
                        self.set_phase(SRV_DRIVE); return
                    return
                self.srv_ok = 0
                if el > g('srv_turn_timeout'):
                    self.abort_dock('SRV_TURN 타임아웃'); return
                w = max(-wmax, min(wmax, kw * hdg))
                if 0 < abs(w) < wmin:
                    w = math.copysign(wmin, w)
                self.send(0.0, w)
                self.get_logger().info(f'SRV_TURN hdg={math.degrees(hdg):+.1f}°',
                                       throttle_duration_sec=0.5)
                return

            if self.phase == SRV_DRIVE:     # 접근점까지 저속 전진(헤딩 미세보정)
                d = math.hypot(tgt[0], tgt[1])
                hdg = math.atan2(tgt[1], tgt[0])
                if d < g('srv_pos_tol'):
                    self.srv_ok += 1; self.send(0.0)
                    if self.srv_ok >= g('srv_ok_count'):
                        self.srv_ok = 0; self.set_phase(SRV_FACE); return
                    return
                self.srv_ok = 0
                if el > g('srv_drive_timeout'):
                    self.abort_dock('SRV_DRIVE 타임아웃'); return
                if abs(hdg) > math.radians(25.0):   # 크게 틀어짐 → 회전부터 다시
                    self.set_phase(SRV_TURN); return
                v = max(0.0, min(g('srv_v_max'), g('srv_k_v') * d))
                w = max(-wmax, min(wmax, kw * hdg))
                self.send(v, w)
                self.get_logger().info(f'SRV_DRIVE d={d*100:.0f}cm hdg={math.degrees(hdg):+.0f}°',
                                       throttle_duration_sec=0.5)
                return

            if self.phase == SRV_FACE:      # ★v3: 마커 '법선각' 정렬(=마커 yaw 반대) → 기존 흐름 합류
                #   bearing(중심응시)은 옆이탈 시 비틀림 → 법선 정렬이면 옆에 있어도 자세 똑바름.
                nrm = self._servo_normal()   # ★WALL_NORMAL: 벽 라이다 우선, 폴백 카메라
                use_n = (g('srv_face_normal') and nrm is not None and
                         math.hypot(nrm[0], nrm[1]) >= g('srv_normal_min'))
                if use_n:
                    # ★기준별 분리(7/9): 벽 법선은 '벽과 직각'이라는 순수 기하 → 카메라 픽셀
                    #   오프셋(bearing_target_deg)을 빼면 안 된다. 카메라 법선/bearing 폴백일 때만 조준 적용.
                    # 카메라 법선 전용 → 화면 중앙 조준(_aim) 적용. 마커를 정중앙에 두고 진입한다.
                    b = self._aim(math.atan2(-nrm[1], -nrm[0]))
                    tag = f'법선({nrm[2]})'
                else:
                    b = self._aim(math.atan2(self.my_f, self.mx_f))      # ★IMG_CENTER 조준. 폴백: 옛 bearing
                    tag = 'bearing'
                if abs(b) < math.radians(g('align_tol_deg')):
                    self.srv_ok += 1; self.send(0.0)
                    if self.srv_ok >= g('align_settle'):
                        self.srv_ok = 0
                        self.via_servo = True   # ★FWD 직진 판단용
                        # ★v4: '완벽정렬 yaw' 기록(잔차 b 제거값) — POST_TRIM이 이걸 기준으로 보정
                        if self.odom_yaw is not None:
                            self.yaw_ref = self._norm_ang(self.odom_yaw - b)
                        # ALIGN 완료와 동일한 라우팅(검증된 흐름 합류)
                        if g('dock_mode') == 'park':
                            if g('srv_range_enable'):
                                self.get_logger().info(
                                    f'★서보정렬 완료 ({tag}오차={math.degrees(b):+.1f}°) → SRV_RANGE(거리 표준화)')
                                self.set_phase(SRV_RANGE); return
                            self.get_logger().info(
                                f'★서보정렬 완료 ({tag}오차={math.degrees(b):+.1f}°) → FWD_APPROACH(전진주차)')
                            self.start_xy = None; self.set_phase(FWD_APPROACH); return
                        if g('use_center_align'):
                            self.center_ok = 0; self.set_phase(CENTER); return
                        self.start_xy = None; self.set_phase(REVERSE); return
                    return
                self.srv_ok = 0
                if el > g('srv_face_timeout'):
                    self.abort_dock(
                        f'SRV_FACE 타임아웃 (기준={tag}, 잔여={math.degrees(b):+.1f}°, '
                        f'tol={g("align_tol_deg")}°)'); return
                w = max(-wmax, min(wmax, kw * b))
                if 0 < abs(w) < wmin:
                    w = math.copysign(wmin, w)
                self.send(0.0, w)
                # ★진단(7/9): 정렬이 왜 안 끝나는지 보이게. 기준(wall/cam)과 잔여각을 남긴다.
                self.get_logger().info(
                    f'SRV_FACE [{tag}] 잔여 {math.degrees(b):+.1f}° w={w:+.3f} ok={self.srv_ok}',
                    throttle_duration_sec=0.5)
                return

        # ---------- ★v4.1 SRV_RANGE (FWD 직전 마커거리 표준화 → 깊이 일정) ----------
        if self.phase == SRV_RANGE:
            g = lambda n: self.get_parameter(n).value
            if not self.marker_fresh(now):
                if el > 3.0:   # 마커 오래 소실 → 그냥 FWD 진행(표준화 포기, 옛동작과 동일)
                    self.get_logger().warn('SRV_RANGE 마커 소실 → 표준화 생략, FWD 진행')
                    self.start_xy = None; self.set_phase(FWD_APPROACH); return
                self.send(0.0); return
            err = self.marker_range_f - g('servo_standoff')   # +면 멀다(전진), -면 가깝다(후진)
            if abs(err) <= g('srv_range_tol'):
                self.srv_ok += 1; self.send(0.0)
                if self.srv_ok >= g('srv_ok_count'):
                    self.srv_ok = 0
                    self.get_logger().info(
                        f'거리 표준화 완료 (마커 {self.marker_range_f:.3f}m ≈ {g("servo_standoff"):.2f}) → FWD_APPROACH')
                    self.start_xy = None; self.set_phase(FWD_APPROACH); return
                return
            self.srv_ok = 0
            if el > g('srv_range_timeout'):
                self.get_logger().warn('SRV_RANGE 타임아웃 → FWD 진행')
                self.start_xy = None; self.set_phase(FWD_APPROACH); return
            v = math.copysign(g('srv_range_speed'), err)
            wb = max(-0.08, min(0.08, 0.3 * self._aim(self.bearing_f)))   # ★IMG_CENTER 정면 유지 미세조향
            self.send(v, wb)
            self.get_logger().info(f'SRV_RANGE 마커 {self.marker_range_f:.3f}→{g("servo_standoff"):.2f}m',
                                   throttle_duration_sec=0.5)
            return

        # ---------- ALIGN (pi_dock 검증 로직) ----------
        if self.phase == ALIGN:
            if not self.marker_fresh(now):
                if el > 1.5:
                    self.get_logger().warn('ALIGN 중 마커 소실 → SEARCH 복귀')
                    self.send(0.0); self.set_phase(SEARCH); self.marker_hits = 0; return
                self.send(0.0); return
            tol = math.radians(self.get_parameter('align_tol_deg').value)
            b = self._aim(self.bearing_f)   # ★IMG_CENTER 조준
            if abs(b) < tol:
                self.align_ok += 1
                self.send(0.0)
                if self.align_ok >= self.get_parameter('align_settle').value:
                    if self.get_parameter('dock_mode').value == 'park':
                        # ★7/9: ALIGN 경로도 거리 표준화를 타게 연결(법선 미사용이라 안전).
                        #   없으면 FWD 30cm가 '도착 거리 편차'를 그대로 최종 깊이로 전달한다.
                        if self.get_parameter('srv_range_enable').value:
                            self.get_logger().info(
                                f'정면정렬 완료 (bear={math.degrees(b):+.1f}°) → SRV_RANGE(거리 표준화)')
                            self.srv_ok = 0; self.set_phase(SRV_RANGE); return
                        self.get_logger().info(f'정면정렬 완료 (bear={math.degrees(b):+.1f}°) → FWD_APPROACH(전진주차)')
                        self.start_xy = None; self.set_phase(FWD_APPROACH); return
                    if self.get_parameter('use_center_align').value:
                        self.get_logger().info(f'정면정렬 완료 (bear={math.degrees(b):+.1f}°) → CENTER(중심선)')
                        self.center_ok = 0; self.set_phase(CENTER); return
                    self.get_logger().info(f'정렬 완료 (bear={math.degrees(b):+.1f}°) → REVERSE')
                    self.start_xy = None; self.set_phase(REVERSE); return
                return
            self.align_ok = 0
            if el > self.get_parameter('align_timeout').value:
                self.abort_dock('ALIGN 타임아웃 — 정렬 실패'); return
            k = self.get_parameter('align_k').value
            wmax = self.get_parameter('align_w_max').value; wmin = self.get_parameter('align_w_min').value
            w = k * b
            w = max(-wmax, min(wmax, w))
            if 0 < abs(w) < wmin:
                w = math.copysign(wmin, w)
            self.send(0.0, w)
            self.get_logger().info(f'ALIGN bear={math.degrees(b):+.1f}° w={w:+.2f}', throttle_duration_sec=0.3)
            return

        # ---------- ★FWD_APPROACH (전진: odom 실주행거리 or 마커거리) ----------
        if self.phase == FWD_APPROACH:
            g = lambda n: self.get_parameter(n).value
            odom_mode = (g('fwd_mode') == 'odom')
            if odom_mode:
                # ★사용자 실측 기준: odom 거리로 전진(마커는 방향만). 마커 놓쳐도 계속 직진.
                if self.start_xy is None:
                    self.start_xy = self.odom_xy
                    self.get_logger().info(f'전진 시작 (odom {g("fwd_dist")*100:.0f}cm 실주행)'); return
                traveled = math.hypot(self.odom_xy[0]-self.start_xy[0], self.odom_xy[1]-self.start_xy[1])
                if traveled >= g('fwd_dist'):
                    self.send(0.0)
                    self.get_logger().info(f'전진 완료 (odom {traveled*100:.1f}cm) → ROTATE_180')
                    self.rot_accum = 0.0; self.rot_last_yaw = self.odom_yaw; self.set_phase(ROTATE_180); return
                if el > g('fwd_timeout'):
                    self.abort_dock('FWD_APPROACH 타임아웃'); return
                w = 0.0                                          # 마커 보이면 중심유지, 아니면 직진
                fresh = self.marker_fresh(now)
                # ★LATERAL_FWD(7/9): 조준(bearing)만으로는 '중심선 위'가 보장되지 않는다.
                #   조준오차를 0으로 만들면 로봇 축이 마커를 향할 뿐, 옆으로 벗어난 채 들어갈 수 있다
                #   (실주행: 도킹존 좌 10cm). 벽법선 기준 횡오차를 함께 물려 곡선으로 중심선에 올라탄다.
                lat = self._lat_off_wall() if (fresh and g('use_lateral_fwd')) else None
                if lat is not None:
                    aim = self._aim(self.bearing_f)
                    wmax = g('lat_fwd_w_max')
                    w = max(-wmax, min(wmax,
                                       g('fwd_k_bearing') * aim
                                       + g('lat_fwd_sign') * g('k_lat_fwd') * lat))
                    self.get_logger().info(
                        f'FWD 복합조향: 조준 {math.degrees(aim):+.1f}° 횡오차 {lat*100:+.1f}cm → w={w:+.3f}',
                        throttle_duration_sec=0.5)
                # ★v3: 서보(법선)정렬로 진입했으면 순수직진 — bearing조향이 다시 중심을
                #   쫓아가며 비틀림을 재생성하는 것 방지.
                elif fresh and not (self.via_servo and g('fwd_straight_after_servo')):
                    w = max(-g('fwd_w_max'), min(g('fwd_w_max'), g('fwd_k_bearing') * self._aim(self.bearing_f)))
                remaining = g('fwd_dist') - traveled
                frac = min(1.0, remaining / g('fwd_slowdown')) if g('fwd_slowdown') > 0 else 1.0
                v = max(g('fwd_min_speed'), g('fwd_speed') * frac)
                self.send(v, w)
                self.get_logger().info(
                    f'FWD odom {traveled*100:.1f}/{g("fwd_dist")*100:.0f}cm '
                    f'bear={math.degrees(self.bearing_f):+.1f}°{"" if fresh else "(마커X 직진)"} v={v:.2f}',
                    throttle_duration_sec=0.3)
                return
            # ---- range 모드(마커거리 기준) ----
            if not self.marker_fresh(now):
                if el > 1.5:
                    self.get_logger().warn('FWD_APPROACH 중 마커 소실 → SEARCH 복귀')
                    self.send(0.0); self.set_phase(SEARCH); self.marker_hits = 0; return
                self.send(0.0); return
            rng = self.marker_range_f
            target = g('fwd_park_range')
            if rng <= target or rng <= g('fwd_range_floor'):
                self.send(0.0)
                self.get_logger().info(f'전진주차 완료 (마커거리 {rng:.3f}m ≤ {target:.3f}m) → ROTATE_180')
                self.rot_accum = 0.0; self.rot_last_yaw = self.odom_yaw; self.set_phase(ROTATE_180); return
            if el > g('fwd_timeout'):
                self.abort_dock('FWD_APPROACH 타임아웃'); return
            w = max(-g('fwd_w_max'), min(g('fwd_w_max'), g('fwd_k_bearing') * self._aim(self.bearing_f)))
            remaining = rng - target
            frac = min(1.0, remaining / g('fwd_slowdown')) if g('fwd_slowdown') > 0 else 1.0
            v = max(g('fwd_min_speed'), g('fwd_speed') * frac)
            self.send(v, w)
            self.get_logger().info(
                f'FWD_APPROACH rng={rng:.3f}m bear={math.degrees(self.bearing_f):+.1f}° v={v:.2f} w={w:+.2f}',
                throttle_duration_sec=0.3)
            return

        # ---------- ★ROTATE_180 (제자리 회전, odom yaw 누적) ----------
        if self.phase == ROTATE_180:
            g = lambda n: self.get_parameter(n).value
            if self.odom_yaw is None:
                self.send(0.0); return
            if self.rot_last_yaw is None:
                self.rot_last_yaw = self.odom_yaw
            dyaw = math.atan2(math.sin(self.odom_yaw - self.rot_last_yaw),
                              math.cos(self.odom_yaw - self.rot_last_yaw))
            self.rot_accum += abs(dyaw)
            self.rot_last_yaw = self.odom_yaw
            if self.rot_accum >= math.radians(g('rotate_deg')) - math.radians(g('rotate_tol_deg')):
                self.send(0.0)
                self.get_logger().info(f'180° 회전 완료 (누적 {math.degrees(self.rot_accum):.1f}°) → BACK_IN')
                self.start_xy = None; self.set_phase(BACK_IN); return
            if el > g('rotate_timeout'):
                self.get_logger().warn(f'ROTATE 타임아웃(누적 {math.degrees(self.rot_accum):.1f}°) → BACK_IN 진행')
                self.send(0.0); self.start_xy = None; self.set_phase(BACK_IN); return
            self.send(0.0, g('rotate_dir') * g('rotate_speed'))
            self.get_logger().info(f'ROTATE 누적 {math.degrees(self.rot_accum):.0f}°/{g("rotate_deg"):.0f}°',
                                   throttle_duration_sec=0.5)
            return

        # ---------- ★BACK_IN (짧은 직진 후진 → 충전기 접점 연결) ----------
        if self.phase == BACK_IN:
            g = lambda n: self.get_parameter(n).value
            if self.start_xy is None:
                self.start_xy = self.odom_xy
                self.back_yaw0 = self.odom_yaw   # ★v4: 후진 시작 yaw(직진 홀드 기준)
                self.get_logger().info(f'후진 시작 (odom {g("back_in_dist")*100:.0f}cm 직진)'); return
            traveled = math.hypot(self.odom_xy[0]-self.start_xy[0], self.odom_xy[1]-self.start_xy[1])
            # ★v5: 라이다 후방벽 정지(절대기준). 게이트 ON+유효시 이게 주 판정,
            #   odom은 하드캡(폭주방지)과 라이다무효 폴백으로 강등.
            done = False
            if g('use_back_lidar'):
                rear = self._rear_wall_dist()
                if rear is not None:
                    self.get_logger().info(f'BACK_LIDAR rear={rear:.3f}m→{g("back_wall_target"):.2f}m',
                                           throttle_duration_sec=0.5)
                    if rear <= g('back_wall_target'):
                        self.get_logger().info(f'★라이다 정지: 후방벽 {rear:.3f}m ≤ {g("back_wall_target"):.2f}m')
                        done = True
                    elif traveled >= g('back_in_dist') + g('back_hard_extra'):
                        self.get_logger().warn(f'BACK_IN odom 하드캡({traveled*100:.1f}cm) — 라이다 목표 미달인데 초과주행 → 정지')
                        done = True
                else:
                    if traveled >= g('back_in_dist'):   # 라이다 무효 → odom 폴백
                        self.get_logger().warn('BACK_IN 라이다 무효 → odom 거리로 정지(폴백)')
                        done = True
            elif traveled >= g('back_in_dist'):
                done = True
            if done:
                if g('use_post_trim') and self.yaw_ref is not None and self.odom_yaw is not None:
                    self.send(0.0)
                    self.get_logger().info(f'후진 완료 ({traveled*100:.1f}cm) → POST_TRIM(자세 미세보정)')
                    self.trim_settle = 0
                    self._wall_hist = []   # ★이전 도킹의 벽각 이력 유입 방지
                    self.set_phase(POST_TRIM); return
                self.get_logger().info(f'충전기 도킹 완료 (후진 {traveled*100:.1f}cm)')
                self.finish('DOCK_DONE'); return
            if el > g('back_in_timeout'):
                self.get_logger().warn('BACK_IN 타임아웃 급정지'); self.finish('DOCK_DONE'); return
            # ★v4: 후진 중 yaw 홀드(비틀림 누적 방지). 끄면 순수직진(옛동작)
            w = 0.0
            if g('back_heading_hold') and self.back_yaw0 is not None and self.odom_yaw is not None:
                yerr = self._norm_ang(self.back_yaw0 - self.odom_yaw)
                w = max(-g('back_w_max'), min(g('back_w_max'), g('back_k_yaw') * yerr))
            self.send(-g('back_in_speed'), w)
            self.get_logger().info(f'BACK_IN {traveled*100:.1f}/{g("back_in_dist")*100:.0f}cm',
                                   throttle_duration_sec=0.5)
            return

        # ---------- ★v4 POST_TRIM (도킹 직후 자세 미세보정: 마커 불필요, odom yaw 기준) ----------
        #   목표 yaw = (FACE완료시 완벽정렬 yaw) + 회전방향*rotate_deg.
        #   FACE 잔차(-1.9° 계통)와 회전 부족분(179.2°)을 한 번에 상쇄.
        if self.phase == POST_TRIM:
            g = lambda n: self.get_parameter(n).value
            # ★v5.1 WALL_TRIM: 라이다 벽각도(절대) 우선, 무효시 odom yaw_ref(상대) 폴백
            yerr = None; mode = 'odom'
            if g('use_wall_trim'):
                phi = self._rear_wall_angle()
                if phi is not None:
                    yerr = -phi; mode = 'wall'   # +φ CCW 틀어짐 → -φ 회전
            if yerr is None:
                target = self._norm_ang(self.yaw_ref + g('rotate_dir') * math.radians(g('rotate_deg')))
                yerr = self._norm_ang(target - self.odom_yaw)
            if abs(yerr) < math.radians(g('post_trim_tol_deg')):
                # ★settle: 정지 상태로 연속 N틱 tol 안이어야 확정(회전 중 스캔왜곡으로 인한 조기종료 방지)
                self.send(0.0)
                self.trim_settle = getattr(self, 'trim_settle', 0) + 1
                if self.trim_settle >= int(g('trim_settle_ticks')):
                    self.get_logger().info(f'★자세보정 완료 [{mode}] (잔여 {math.degrees(yerr):+.2f}°) → 도킹 확정')
                    self.finish('DOCK_DONE'); return
                return
            self.trim_settle = 0
            if el > g('post_trim_timeout'):
                self.get_logger().warn(f'POST_TRIM 타임아웃 [{mode}] (잔여 {math.degrees(yerr):+.1f}°) — 도킹 확정')
                self.finish('DOCK_DONE'); return
            # ★7/9: P제어 — 오차에 비례, 상한 클램프, 정지마찰 최소값 바닥.
            wmax = g('post_trim_speed')
            w = max(-wmax, min(wmax, g('post_trim_k') * yerr))
            wmin = g('post_trim_w_min')
            if 0 < abs(w) < wmin:
                w = math.copysign(wmin, w)
            self.send(0.0, w)
            self.get_logger().info(f'POST_TRIM [{mode}] 잔여 {math.degrees(yerr):+.1f}° w={w:+.3f}',
                                   throttle_duration_sec=0.4)
            return

        # ---------- ★CENTER (중심선 정렬: 호 그려 lat_off=0 + 정면 유지) ----------
        if self.phase == CENTER:
            g = lambda n: self.get_parameter(n).value
            if not self.marker_fresh(now):
                if el > 1.5:
                    self.get_logger().warn('CENTER 중 마커 소실 → SEARCH 복귀')
                    self.send(0.0); self.set_phase(SEARCH); self.marker_hits = 0; self.center_ok = 0; return
                self.send(0.0); return
            lat = self.lat_off_f if self.lat_off_f is not None else 0.0
            b = self._aim(self.bearing_f)   # ★IMG_CENTER 조준
            rng = self.marker_range_f if self.marker_range_f is not None else 1.0
            lat_tol = g('center_lat_tol')
            bear_tol = math.radians(g('align_tol_deg'))
            # 양축(중심선 lat=0 + 정면 bearing=0) 모두 만족 → REVERSE
            if abs(lat) < lat_tol and abs(b) < bear_tol:
                self.center_ok += 1
                self.send(0.0)
                if self.center_ok >= g('center_settle'):
                    self.get_logger().info(
                        f'중심선 정렬 완료 (lat={lat*100:+.1f}cm bear={math.degrees(b):+.1f}°) → REVERSE')
                    self.start_xy = None; self.set_phase(REVERSE); return
                return
            self.center_ok = 0
            if el > g('center_timeout'):
                self.get_logger().warn(
                    f'CENTER 타임아웃(lat={lat*100:+.1f}cm) — 정렬포기, REVERSE 서보폴백')
                self.start_xy = None; self.set_phase(REVERSE); return
            # 조향 = 정면유지(k_bear·bearing) + 중심선보정(sign·k_lat·lat). 부호는 전진기준(center_lat_sign).
            w = g('center_k_bear') * b + g('center_lat_sign') * g('center_k_lat') * lat
            wm = g('center_w_max')
            w = max(-wm, min(wm, w))
            # 전진: 중심선 이탈 클 때만 + 마커가 너무 가깝지 않을 때만(벽충돌 방지)
            v = g('center_speed') if (abs(lat) > lat_tol and rng > g('center_range_floor')) else 0.0
            self.send(v, w)
            self.get_logger().info(
                f'CENTER lat={lat*100:+.1f}cm bear={math.degrees(b):+.1f}° rng={rng:.2f}m v={v:.2f} w={w:+.2f}',
                throttle_duration_sec=0.3)
            return

        # ---------- REVERSE (검증된 pi_reverse 로직) ----------
        if self.phase == REVERSE:
            by_range = self.get_parameter('reverse_by_range').value
            target_rng = self.get_parameter('target_range').value
            if self.start_xy is None:
                self.start_xy = self.odom_xy
                # ★시작 시 이미 도킹거리 도달(pre-dock이 벽에 너무 가까움)이면 후진 안 함(벽충돌 방지)
                if by_range and self.marker_fresh(now) and self.marker_range_f >= target_rng:
                    self.get_logger().warn(
                        f'시작부터 마커거리 {self.marker_range_f:.3f}m ≥ 목표 {target_rng:.3f}m '
                        f'— 이미 도킹위치, 후진 생략'); self.finish('DOCK_DONE'); return
                self.get_logger().info(f'후진 시작 (마커거리기준, 목표 {target_rng:.3f}m)'); return
            traveled = math.hypot(self.odom_xy[0] - self.start_xy[0], self.odom_xy[1] - self.start_xy[1])
            # 안전캡(어떤 경우든): odom distance+margin 넘으면 급정지(마커 놓쳐도 벽 못 감)
            if traveled >= self.distance + self.hard_margin:
                self.get_logger().warn(f'안전캡 급정지! odom {traveled*100:.1f}cm'); self.finish('DOCK_DONE'); return
            if el > self.max_time:
                self.get_logger().warn('후진 최대시간 급정지'); self.finish('DOCK_DONE'); return
            # ★주 정지조건: 마커거리 도달 (시작점 변동 흡수)
            if by_range and self.marker_fresh(now) and self.marker_range_f >= target_rng:
                self.get_logger().info(
                    f'도킹 완료(마커거리 {self.marker_range_f:.3f}m ≥ {target_rng:.3f}m, odom {traveled*100:.1f}cm)')
                self.finish('DOCK_DONE'); return
            # 폴백: 마커 놓쳤거나 odom모드 → odom distance로 정지
            if (not by_range or not self.marker_fresh(now)) and traveled >= self.distance:
                self.get_logger().info(f'도킹 완료(odom {traveled*100:.1f}cm, 마커 {"미사용" if not by_range else "소실"})')
                self.finish('DOCK_DONE'); return
            w = 0.0
            if self.get_parameter('use_marker_steer').value and self.marker_fresh(now):
                kb = self.get_parameter('k_bearing').value
                w = self.get_parameter('reverse_steer_sign').value * kb * self.bearing_f
                # ★가로 서보: 중심선 이탈(lat_off_f)을 0으로 몰아감
                if self.get_parameter('use_lateral_servo').value and self.lat_off_f is not None:
                    kl = self.get_parameter('k_lateral').value
                    ls = self.get_parameter('lateral_sign').value
                    lt = self.get_parameter('lateral_target').value
                    w += ls * kl * (self.lat_off_f - lt)
                wm = self.get_parameter('w_max').value
                w = max(-wm, min(wm, w))
                self.get_logger().info(
                    f'후진조향 bear={math.degrees(self.bearing_f):+.1f}° '
                    f'lat={self.lat_off_f*100 if self.lat_off_f is not None else 0:+.1f}cm w={w:+.2f}',
                    throttle_duration_sec=0.5)
            remaining = self.distance - traveled
            frac = min(1.0, remaining / self.slowdown) if self.slowdown > 0 else 1.0
            mag = max(self.min_speed, self.speed * frac)
            self.send(-mag, w)
            self.get_logger().info(f'후진 {traveled*100:.1f}/{self.distance*100:.0f}cm', throttle_duration_sec=0.25)
            return


def main():
    import signal
    import time as _t
    rclpy.init()
    node = PiDockExecutor()

    # ★안전: SIGTERM/SIGINT를 잡아 종료 시 0속도 확실히 발행(런어웨이 방지).
    #   (SIGKILL은 못 잡으니 절대 kill -9 금지 — SIGTERM으로 정지할 것)
    def _stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 확실히 정지: 신선 스탬프 0속도를 0.02s 간격으로 15회(~0.3s) 지속 발행.
        try:
            for _ in range(15):
                node.send(0.0, 0.0)
                _t.sleep(0.02)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
