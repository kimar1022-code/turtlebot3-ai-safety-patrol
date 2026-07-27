#!/usr/bin/env bash
# =============================================================================
# KILL_ALL.sh — 전체 강제종료만 (CLEAN_START의 0단계와 동일, 기동은 안 함)
# 사용: bash ~/team_ws/KILL_ALL.sh
# 언제 쓰나: 노드가 꼬였는데 재기동은 나중에 하고 싶을 때 / 로봇 정리하고 퇴근할 때.
# 재기동까지 하려면 이거 말고 CLEAN_START.sh를 쓰면 됨(안에 이 내용 포함).
# =============================================================================
PI=codelab@192.168.40.101

echo "=== PC: 순찰/도킹/검출 노드 강제종료 ==="
pkill -9 -f 'lib/teamproject_navigation/patrol_commander|lib/teamproject_navigation/status_reporter' 2>/dev/null
pkill -9 -f 'component_container' 2>/dev/null
pkill -9 -f 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor' 2>/dev/null
pkill -9 -f 'nav2_amcl|/amcl|nav2_map_server|/map_server|filter_mask_server|costmap_filter_info|lifecycle_manager' 2>/dev/null
pkill -9 -f 'udp_camera_bridge|aruco_dock_detector|dock_detector_manager|aruco_localizer|safety_dispatch_fusion' 2>/dev/null
pkill -9 -f 'id2_follow_detector|camera_optical_tf|manual_cmd_gate' 2>/dev/null
pkill -9 -f 'publish_camera_info|compressed_to_raw' 2>/dev/null
pkill -9 -f 'navigation2.launch|nav2_norviz.launch|robot_nodes.launch|keepout_filter' 2>/dev/null
pkill -9 -f 'rviz2' 2>/dev/null
pkill -9 -f 'rqt_image_view|rqt_gui' 2>/dev/null

echo "=== Pi(로봇): 브링업/라이다/카메라/도킹 강제종료 ==="
# 주의: 패턴에 bracket([r]obot_...)을 써서 원격 bash 자기매칭 방지(안 쓰면 ssh가 자기 셸을 죽여 255)
# coin_d4 = 실사용 라이다 드라이버(single_coin_d4_node). ld08/hlds는 옛 모델명(호환 유지).
ssh -o ConnectTimeout=10 "$PI" \
  "pkill -9 -f '[r]obot_bringup|[p]i_dock_executor|[t]urtlebot_udp_camera|[t]urtlebot3_node|[c]oin_d4|[l]d08|[h]lds|[d]iff_drive|[r]obot_state_publisher'" 2>/dev/null

echo "=== ros2 daemon 재시작 ==="
pkill -9 -f '_ros2_daemon' 2>/dev/null
sleep 2

echo ""
echo "=== 확인 (전부 0이어야 깨끗) ==="
echo "PC 잔존:  patrol=$(pgrep -fc '[p]atrol_commander')  nav2=$(pgrep -fc '[c]ontroller_server')  container=$(pgrep -fc '[c]omponent_container')  rviz=$(pgrep -fc '[r]viz2')  fusion=$(pgrep -fc '[s]afety_dispatch_fusion')  bridge=$(pgrep -fc '[u]dp_camera_bridge')"
echo "Pi 잔존:  $(ssh -o ConnectTimeout=5 "$PI" "echo core=\$(pgrep -fc '[t]urtlebot3_node') lidar=\$(pgrep -fc '[c]oin_d4') camera=\$(pgrep -fc '[t]urtlebot_udp_camera_sender') dock=\$(pgrep -fc '[p]i_dock_executor')" 2>/dev/null || echo 'SSH 실패')"
echo ""
echo "✅ 전체 강제종료 완료. 다시 켜려면: bash ~/team_ws/CLEAN_START.sh"
