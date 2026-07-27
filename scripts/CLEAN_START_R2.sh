#!/usr/bin/env bash
# 2호기 클린 기동 — 사용법: bash ~/team_ws/CLEAN_START_R2.sh
# ⚠️ 실행 전 채울 것: PI(2호기 라즈베리파이 SSH 주소). 나머지는 확정값.
export ROBOT_ID=2
export DOMAIN=88          # 2호기 도메인 (1호기=97과 반드시 달라야 함)
export PI="pi@192.168.40.102"   # ★7/14 확정: 로봇2 계정=pi (1호기 codelab과 다름!)
export CAM_PORT=5008
export DOCK_MARKER_ID=43
export DOCK_X=0.0 DOCK_Y=0.26 DOCK_YAW=0.0
# ★FUSION_CALIB(7/15): 로봇2 카메라 캘리브(ost_r2.yaml fx1304.2/cx316.5) 기반 이벤트 fusion 보정.
#   기본값(28.91/1.91)은 로봇1 전용이라 로봇2 방위각이 ~2° 틀어져 이벤트 접근/인식 실패 → 로봇2값 주입.
export FUSION_HFOV_DEG=27.6            # 2*atan(320/1304.2)=27.59°
export FUSION_BEARING_OFFSET_DEG=-0.15 # cx316.5(중앙 320 대비 -3.5px) → ~-0.15°(로봇1 +1.91°와 반대)
export ROUTE_GRAPH=~/team_ws/maps/patrol_graph_r2.geojson   # ★7/14: 로봇2 전용(프리도킹 노드13 y+0.26)   # ★7/14 확정: 몸체간격 8cm 실측 → 중심간 0.26m (AMCL 수렴값 일치)   # ★7/14: 로봇2 도킹 마커 (로봇1=42와 분리, aruco_robot2_dock_id43.png)      # 1호기=5007. Pi 카메라 송신기도 이 포트로 쏘게 설정!
# export MAP=...          # 다른 맵 쓰면 주석 해제
# export PARAMS=...
source "$HOME/team_ws/CLEAN_START_COMMON.sh"
