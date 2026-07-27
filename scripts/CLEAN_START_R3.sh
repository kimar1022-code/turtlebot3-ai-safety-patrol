#!/usr/bin/env bash
# 3호기 클린 기동 — 사용법: bash ~/team_ws/CLEAN_START_R3.sh
# ⚠️ 실행 전 채울 것: PI(3호기 라즈베리파이 SSH 주소). 나머지는 확정값.
export ROBOT_ID=3
export DOMAIN=4           # 3호기 도메인
export PI="codelab@CHANGE_ME_3호기_PI_IP"
export CAM_PORT=5009      # 1호기=5007, 2호기=5008과 겹치지 않게
# export MAP=...
# export PARAMS=...
source "$HOME/team_ws/CLEAN_START_COMMON.sh"
