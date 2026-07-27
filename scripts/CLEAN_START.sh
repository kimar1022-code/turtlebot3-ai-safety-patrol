#!/usr/bin/env bash
# =============================================================================
# CLEAN_START.sh — 1호기 클린 기동 (도메인 97)
#
# ★7/17 사용자 지시("1호기 2호기 똑같이 맞춰")로 CLEAN_START_R2.sh 와 동일한
#   얇은 래퍼 + CLEAN_START_COMMON.sh 구조로 교체.
#   백업(옛 자립형 스크립트): CLEAN_START.sh.BEFORE_COMMON_ALIGN_0717
#
# ★교체 이유 = 7/15·7/17 두 번 재현된 사고:
#   옛 스크립트는 [0]단계가 전역 `pkill -9 -f '...patrol_commander...'` 라
#   도메인을 안 가리고 **로봇2 노드까지 몰살**했다(7/17 10:50:57 실측: R2 patrol
#   exit -9, 이벤트 촬영 도중 사망). 7/14 SAME_PC 도메인 스코프 킬은 COMMON 에만
#   들어갔고 1호기 스크립트는 옛 세대로 방치돼 있었다.
#   COMMON 은 /proc/PID/environ 의 ROS_DOMAIN_ID 로 자기 로봇만 골라 죽인다.
#
# ※ COMMON 의 기본값이 원래 1호기 값(hfov 28.91 / offset 1.91 / marker 42 /
#   patrol_graph.geojson / dock 0,0,0)이지만, 2호기 래퍼와 형태를 맞추고
#   "기본값이 조용히 바뀌어도 1호기는 안 흔들리게" 명시적으로 못박아 둔다.
# ※ 카메라: robot_bringup 이 Pi 에서 게이트(camera_gate.py)+센더를 띄운다.
#   COMMON [5]단계는 기동이 아니라 '토픽 수신 확인'만 한다(옛 CAM_IN_BRINGUP=1 과 동일 동작).
# ※ 로그 위치 변경: ~/team_ws/run_logs  →  ~/team_ws/run_logs_r1
# =============================================================================
export ROBOT_ID=1
export DOMAIN=97                      # 1호기 도메인 (2호기=88과 반드시 달라야 함)
export PI="codelab@192.168.40.101"    # 1호기 Pi 계정=codelab (2호기 pi 와 다름!)
export CAM_PORT=5007                  # 2호기=5008
export DOCK_MARKER_ID=42              # 2호기=43
export DOCK_X=0.0 DOCK_Y=0.0 DOCK_YAW=0.0        # 1호기 도크=원점 (2호기=(0,0.26))
export FUSION_HFOV_DEG=28.91                     # 7/9 캘리브 fx1268.3 → 2*atan(320/1268.3)
export FUSION_BEARING_OFFSET_DEG=1.91            # 7/9 캘리브 cx362.3(+42px) → +1.91°
export ROUTE_GRAPH=~/team_ws/maps/patrol_graph.geojson   # 2호기는 patrol_graph_r2(노드13 y+0.26)

source "$HOME/team_ws/CLEAN_START_COMMON.sh"
