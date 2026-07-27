#!/usr/bin/env bash
# =============================================================================
# CLEAN_START_COMMON.sh — 멀티로봇 공용 클린 기동 본체 (2026-07-09 게이트식 v6)
#
# 직접 실행하지 말 것! 로봇별 래퍼로 실행:
#   bash ~/team_ws/CLEAN_START_R2.sh   (2호기, 도메인 88)
#
# ★v6 변경: 로봇1에서 실증된 "순차 게이트식"으로 재구성 (7/8 로봇1 검증분 포팅).
#   구버전(고정 sleep)은 동시부하로 WiFi 핑폭발 + odom TF 레이스(Nav2 abort) 유발.
#   → "하나 완전히 뜬 걸 확인한 뒤 다음": scan→odomTF→핑 게이트 후에야 Nav2 기동,
#     Nav2 active 확인(실패 시 자동 재-STARTUP) 후에야 다음 단계.
#
# ★래퍼가 정해줘야 하는 변수 (필수):
#   ROBOT_ID / DOMAIN / PI(user@ip) / CAM_PORT
# ★선택 변수(기본값 있음): PI_WS, PI_UNICAST_XML, MAP, PARAMS
# =============================================================================
: "${ROBOT_ID:?래퍼에서 ROBOT_ID를 지정하세요}"
: "${DOMAIN:?래퍼에서 DOMAIN을 지정하세요}"
: "${PI:?래퍼에서 PI(user@ip)를 지정하세요}"
: "${CAM_PORT:?래퍼에서 CAM_PORT를 지정하세요}"
PI_WS="${PI_WS:-~/turtlebot3_ws}"
PI_USER_HOME="/home/${PI%%@*}"
PI_UNICAST_XML="${PI_UNICAST_XML:-${PI_USER_HOME}/cyclonedds_unicast.xml}"
MAP="${MAP:-$HOME/team_ws/maps/map.yaml}"
PARAMS="${PARAMS:-$HOME/nav_params/burger_rpp.yaml}"

NS="/robot${ROBOT_ID}"
ROBOT_IP="${PI##*@}"
export ROS_DOMAIN_ID="$DOMAIN"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclonedds_unicast.xml"
# ★bashrc 오염 방어(7/8 실증): RANGE=OFF가 있으면 PC 로컬노드 discovery 사망
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source "$HOME/team_ws/install/setup.bash"
LOG="$HOME/team_ws/run_logs_r${ROBOT_ID}"; mkdir -p "$LOG"
# ★SAME_PC(7/14, 사용자: "어느 PC에서든"): 카메라 PC스트림(dock_host)을 '이 스크립트를 실행한
#   PC'로 자동 지정 — 40.x 대역 내 IP 자동 감지. 번들 선기입(.4) 하드코딩 대체.
PC_CAM_HOST="${PC_CAM_HOST:-$(hostname -I | tr ' ' '\n' | grep '^192\.168\.40\.' | head -1)}"
echo "   카메라 PC스트림 대상: $PC_CAM_HOST (자동감지)"
S(){ ssh -o ConnectTimeout=10 -o ServerAliveInterval=4 "$PI" "$@"; }

echo "===== ${ROBOT_ID}호기 게이트식 클린 기동 (도메인 $DOMAIN, $NS, Pi $PI, 카메라포트 $CAM_PORT) ====="

# ---- 게이트 헬퍼: gate <설명> <타임아웃s> -- <성공판정 명령> ----
gate(){
  local desc="$1" tmax="$2"; shift 2
  [ "$1" = "--" ] && shift
  local t=0
  printf "   ⏳ %s " "$desc"
  while [ "$t" -lt "$tmax" ]; do
    if "$@" >/dev/null 2>&1; then printf " ✅(%ss)\n" "$t"; return 0; fi
    sleep 3; t=$((t+3)); printf "."
  done
  printf " ⚠️타임아웃(%ss) — 계속 진행(수동확인 필요)\n" "$tmax"; return 1
}
chk_scan(){   timeout 5 ros2 topic hz /scan 2>&1 | grep -q "average rate"; }
chk_odomtf(){ timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation"; }
chk_ping(){   ping -c 3 -W 1 "$ROBOT_IP" 2>&1 | grep -q " 0% packet loss"; }
chk_nav(){    for n in controller_server planner_server bt_navigator; do
                timeout 5 ros2 lifecycle get /$n 2>&1 | grep -q "active" || return 1; done; }
chk_node(){   timeout 6 ros2 node list 2>&1 | grep -q "$1"; }
chk_topic(){  timeout 6 ros2 topic hz "$1" 2>&1 | grep -q "average rate"; }

# ---- 이전 CLEAN_START 인스턴스 자동 종료 (좀비/중복 방지, PID 락) ----
LOCKF="/tmp/clean_start_r${ROBOT_ID}_${USER}.pid"
if [ -f "$LOCKF" ]; then
  OLD="$(cat "$LOCKF" 2>/dev/null)"
  if [ -n "$OLD" ] && [ "$OLD" != "$$" ] && kill -0 "$OLD" 2>/dev/null \
     && tr '\0' ' ' < "/proc/$OLD/cmdline" 2>/dev/null | grep -q 'CLEAN_START'; then
    echo "이전 CLEAN_START(PID $OLD) 실행중 → 자동 종료"
    pkill -9 -P "$OLD" 2>/dev/null
    kill -9 "$OLD" 2>/dev/null
    sleep 1
  fi
fi
echo "$$" > "$LOCKF"

echo "===[0] 이 로봇(도메인 $DOMAIN) 소속만 강제종료 (★SAME_PC 7/14) ==="
# ★SAME_PC(7/14, 사용자: "한 PC에서 로봇1·2 동시에"): 무차별 pkill이 다른 로봇 스택까지
#   몰살하던 것을 도메인 스코프 킬로 교체 — 프로세스 환경변수 ROS_DOMAIN_ID 로 판별.
#   도메인 표식 없는 프로세스는 안 죽인다(다른 로봇 보호 우선, 잔재는 수동 정리).
kill_domain(){
  local pat="$1"
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$DOMAIN"; then
      kill -9 "$pid" 2>/dev/null
    fi
  done
}
kill_domain 'lib/teamproject_navigation/patrol_commander|lib/teamproject_navigation/status_reporter'
kill_domain 'component_container'
kill_domain 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor'
kill_domain 'nav2_amcl|/amcl|nav2_map_server|/map_server|filter_mask_server|costmap_filter_info|lifecycle_manager'
kill_domain 'udp_camera_bridge|aruco_dock_detector|dock_detector_manager|aruco_localizer|safety_dispatch_fusion'
kill_domain 'id2_follow_detector|camera_optical_tf|manual_cmd_gate'
kill_domain 'publish_camera_info|compressed_to_raw'
kill_domain 'navigation2.launch|nav2_norviz.launch|robot_nodes.launch|keepout_filter'
kill_domain 'rviz2|rqt_image_view|rqt_gui'
S "pkill -9 -f 'robot_bringup|pi_dock_executor|pi_lift_servo|turtlebot_udp_camera|turtlebot3_node|ld08|hlds|diff_drive|robot_state_publisher'" 2>/dev/null || true
sleep 3
echo "   ros2 daemon 재시작"
pkill -9 -f '_ros2_daemon' 2>/dev/null || true
sleep 2

echo "===[1] Pi 브링업 (코어+라이다+카메라+도킹executor) ==="
# 카메라는 브링업에 포함(cam_port 기본값=로봇별 launch에 선기입). 센더 자체에 중복방지 self-guard 내장(v6).
S "export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$PI_UNICAST_XML TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-03; \
   source /opt/ros/jazzy/setup.bash; source $PI_WS/install/setup.bash; \
   setsid ros2 launch teamproject_robot_bringup robot_bringup.launch.py dock_host:=$PC_CAM_HOST > ~/bringup.log 2>&1 < /dev/null &"
gate "scan 흐름"       40 -- chk_scan
gate "odom TF"         30 -- chk_odomtf
gate "핑 안정(0%손실)"  30 -- chk_ping

echo "===[2] Nav2 (odom TF 확인 후 기동 → abort 방지) ==="
setsid ros2 launch teamproject_navigation nav2_norviz.launch.py use_sim_time:=false \
  map:="$MAP" params_file:="$PARAMS" > "$LOG/nav2.log" 2>&1 < /dev/null &
if ! gate "Nav2 활성화(controller/planner/bt active)" 45 -- chk_nav; then
  echo "   ↻ navigation 라이프사이클 자동 재-STARTUP (odom TF 레이스 복구)..."
  timeout 40 ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1
  gate "Nav2 활성화 재확인" 30 -- chk_nav
fi

echo "===[3] KeepoutFilter ==="
setsid ros2 launch teamproject_navigation keepout_filter_newmap.launch.py > "$LOG/keepout.log" 2>&1 < /dev/null &
gate "filter_mask_server 노드" 20 -- chk_node filter_mask_server

echo "===[4] 순찰노드 + 융합 + 카메라브릿지 + 검출 (전부 robot_id=${ROBOT_ID}) ==="
setsid ros2 launch teamproject_navigation robot_nodes.launch.py robot_id:=${ROBOT_ID} dock_x:=${DOCK_X:-0.0} dock_y:=${DOCK_Y:-0.0} dock_yaw:=${DOCK_YAW:-0.0} route_graph:=${ROUTE_GRAPH:-~/team_ws/maps/patrol_graph.geojson} > "$LOG/patrol.log" 2>&1 < /dev/null &
gate "patrol_commander 노드" 25 -- chk_node patrol_commander
setsid python3 ~/team_ws/aruco_docking/udp_camera_bridge.py --ros-args \
  -p port:=${CAM_PORT} -p output_topic:=${NS}/camera/image_raw/compressed \
  > "$LOG/bridge.log" 2>&1 < /dev/null &
# ★R2_DOCK(7/14): 로봇별 도킹 마커 분리 — 로봇1=42(기본), 로봇2=43 (R2 스크립트가 export)
setsid python3 ~/team_ws/aruco_docking/aruco_dock_detector.py --ros-args \
  -p image_topic:=${NS}/camera/image_raw/compressed -p camera_info_topic:=${NS}/camera/camera_info \
  -p marker_id:=${DOCK_MARKER_ID:-42} \
  > "$LOG/detector.log" 2>&1 < /dev/null &
setsid python3 ~/team_ws/aruco_docking/safety_dispatch_fusion.py --ros-args \
  -p robot_id:=${ROBOT_ID} \
  -p hfov_deg:=${FUSION_HFOV_DEG:-28.91} \
  -p bearing_offset_deg:=${FUSION_BEARING_OFFSET_DEG:-1.91} \
  > "$LOG/fusion.log" 2>&1 < /dev/null &
gate "safety_dispatch_fusion 노드" 15 -- chk_node safety_dispatch_fusion
# ★카메라 캘리브 결과가 있으면 camera_info(실측 K/D) 발행 — 없으면 검출기가 근사K로 동작(캘리브 권장!)
# ★CALIB_R2(7/15): 로봇별 캘리브 우선(ost_r${ROBOT_ID}.yaml), 없으면 공용 ost.yaml 폴백
CAL_YAML="$HOME/team_ws/calib/ost_r${ROBOT_ID}.yaml"
[ -f "$CAL_YAML" ] || CAL_YAML="$HOME/team_ws/calib/ost.yaml"
if [ -f "$CAL_YAML" ]; then
  echo "   camera_info 캘리브: $CAL_YAML"
  setsid python3 ~/team_ws/aruco_docking/publish_camera_info.py --yaml "$CAL_YAML" \
    --topic ${NS}/camera/camera_info > "$LOG/caminfo.log" 2>&1 < /dev/null &
  echo "   camera_info(캘리브 K) 발행 기동"
else
  echo "   ⚠️ ~/team_ws/calib/ost.yaml 없음 — 카메라 캘리브 후 배치하면 자동 발행됨(README 6장)"
fi

echo "===[5] 카메라 수신 확인 ==="
gate "카메라 이미지 토픽" 25 -- chk_topic ${NS}/camera/image_raw/compressed

echo "===[6] RViz (GUI 세션에서만) ==="
RVIZ_CFG="$HOME/turtlebot3_ws/install/turtlebot3_navigation2/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz"
[ -f "$RVIZ_CFG" ] && setsid rviz2 -d "$RVIZ_CFG" > "$LOG/rviz.log" 2>&1 < /dev/null &
sleep 2

# ★HANDOVER 릴레이 제거(7/17 사용자 확정): "릴레이 빼자 — 서버에서 이미 교대를 만들어 놨고,
#   방금 두 대가 동시에 나가는 불상사가 있었다."
#   → 교대 주체가 서버 하나로 일원화. PC 릴레이를 같이 띄우면 handover_request 를 둘이
#     받아 각자 PATROL_START 를 쏘므로 두 대 동시 출동(=충돌 위험)이 난다.
#   로봇측은 그대로: 랩+도킹 완료 시 /robotN/handover_request 발행(use_lap_swap)만 하고,
#   그걸 받아 상대를 투입하는 건 서버 몫이다.
#   되살리려면(서버 교대가 없을 때만!): 아래 두 줄 주석 해제.
#   백업: CLEAN_START_COMMON.sh.BEFORE_RELAY_REMOVE_0717 / 릴레이 본체는 handover_relay.py 유지.
# echo "===[7] 교대 릴레이 (robot1↔robot2, PC당 1개 싱글턴) ==="
# bash "$HOME/team_ws/START_HANDOVER_RELAY.sh" || echo "   ⚠️ 교대 릴레이 기동 실패 — 교대 없이 단독 순찰만 됨"

echo ""
echo "=== 최종 점검 ==="
echo "-- 중복노드(아무것도 안나와야 정상):"; timeout 8 ros2 node list 2>/dev/null | sort | uniq -d
echo "-- 카메라 센더(Pi, 1이어야):    $(S "pgrep -fc '[t]urtlebot_udp_camera_sender'" 2>/dev/null)"
printf -- "-- Nav2: "; for n in controller_server planner_server bt_navigator; do
  printf "%s=%s " "$n" "$(timeout 4 ros2 lifecycle get /$n 2>&1 | awk '{print $1}')"; done; echo ""
echo ""
echo "✅ ${ROBOT_ID}호기 순차 기동 완료.  로그: $LOG/*.log , Pi: ~/bringup.log"
echo "다음: RViz에서 (0,0) 위치추정  또는  PATROL_START:"
echo "   ros2 service call ${NS}/set_mode teamproject_interfaces/srv/SetMode \"{mode: 'PATROL_START'}\""
