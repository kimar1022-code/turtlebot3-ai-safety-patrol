# 로봇3 배포 번들 (2026-07-14) — 순찰 + 서버 수동조작

주기능: **자율 순찰(랩+도킹)** + **서버(관제) 수동조작**. 로봇1·2에서 실전 검증된 7/14자 최신 코드 전체입니다.

## 0. 확정값 (로봇3)
| 항목 | 값 | 비고 |
|---|---|---|
| ROS 도메인 | **4** | 1호기=97, 2호기=88 — 절대 겹치면 안 됨(겹치면 서로 붕괴) |
| 네임스페이스 | /robot3 | robot_id:=3이 자동 부여 |
| 카메라 UDP 포트 | **5009** | 1호기=5007, 2호기=5008 |
| 도킹 마커 | **ID 44** | docs/aruco_robot3_dock_id44.png — 마커 본체 12.0cm로 인쇄, 중심높이 16cm(카메라 높이), 수직·정면 부착 |
| 도크 좌표 | [TODO] | CLEAN_START_R3.sh의 DOCK_X/Y/YAW에 실측 기입 |

## 1. PC 설치 (관제 PC — 아무 PC나 가능)
```bash
# 1) 작업공간 구성
mkdir -p ~/team_ws/src ~/nav_params
cp -r pc/teamproject_navigation pc/teamproject_interfaces ~/team_ws/src/
cp -r pc/aruco_docking pc/calib ~/team_ws/
cp -r maps ~/team_ws/maps
cp pc/burger_rpp.yaml ~/nav_params/
cp pc/CLEAN_START_R3.sh pc/CLEAN_START_COMMON.sh ~/team_ws/ && chmod +x ~/team_ws/CLEAN_START_*.sh
# 2) DDS 유니캐스트 설정 (⚠️ WiFi 안정성의 핵심)
cp pc/cyclonedds_unicast_PC.xml ~/cyclonedds_unicast.xml
#    → 파일 열어 NetworkInterface name= 을 이 PC의 40.x 인터페이스명으로 교체 (ip -br addr 로 확인)
#    → Peers에 이 PC IP·로봇3 Pi IP 추가 (참여 안 하는 IP는 있어도 무해)
# 3) bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds_unicast.xml' >> ~/.bashrc
#    ⚠️ ROS_AUTOMATIC_DISCOVERY_RANGE=OFF 가 bashrc에 있으면 삭제 (로컬 discovery 죽는 주범)
# 4) 빌드
cd ~/team_ws && colcon build --packages-select teamproject_interfaces && colcon build --packages-select teamproject_navigation && source install/setup.bash
```

## 2. Pi 설치 (로봇3 라즈베리파이)
```bash
# 1) 파일 배치 (Pi 홈에)
pi/pi_dock_executor.py            → ~/pi_dock_executor.py       # 도킹 실행기 (7/14 최신: 벽피팅+yaw보정 노브)
pi/turtlebot_udp_camera_sender.py → ~/turtlebot_udp_camera_sender.py
pi/cyclonedds_unicast_PI.xml      → ~/cyclonedds_unicast.xml    # Peers에 관제PC IP·자기 IP 반영
pi/robot_bringup.launch.py.TEMPLATE → 브링업 패키지의 launch로 (아래 수정 후)
# 2) TEMPLATE 수정 포인트 (로봇2용이므로 3용으로 치환)
#    - cmd_topic:=/robot2/dock_cmd → /robot3/dock_cmd (done_topic도 동일하게)
#    - cam_port default 5008 → 5009
#    - wall_yaw_offset_deg:=5.0 → 0.0으로 시작 (도킹이 삐뚤면 그때 실측각 기입 — 로봇마다 라이다 장착각 다름)
#    - search_speed:=0.06 유지 (카메라 10fps 기준 탐색속도)
#    - id2_follow / dispatch 토픽의 robot2 → robot3
# 3) bashrc: export ROS_DOMAIN_ID=4, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 4) ⚠️⚠️ WiFi 절전 영구 OFF (안 하면 링크가 서서히 죽음 — 로봇1·2 둘 다 실증된 필수 조치)
sudo bash -c 'printf "[connection]\nwifi.powersave = 2\n" > /etc/NetworkManager/conf.d/wifi-powersave-off.conf' && sudo systemctl restart NetworkManager
# 5) PC→Pi 무비번 SSH: PC에서 ssh-copy-id <계정>@<Pi IP>
# 6) 브링업 빌드: cd ~/turtlebot3_ws && colcon build --packages-select teamproject_robot_bringup
```

## 3. 기동 & 사용
```bash
bash ~/team_ws/CLEAN_START_R3.sh     # 게이트식 순차 기동 (Pi브링업→scan→Nav2→금지존→순찰→카메라 검증)
# 순찰 시작 (로봇이 자기 도크 테이프 위에 있는 상태에서!):
ros2 service call /robot3/set_mode teamproject_interfaces/srv/SetMode "{mode: 'PATROL_START'}"
#   → 위치 자동주입(DOCK_X/Y/YAW) → 언도크 → 랩 → 자동 도킹 → 대기(STAY_DOCKED: 신호 줄 때까지 안 나옴)
# 수동조작(주기능): 서버가 /robot3/cmd_vel(TwistStamped)로 발행 → MANUAL_CONTROL 모드에서만 모터로 통과
ros2 service call /robot3/set_mode teamproject_interfaces/srv/SetMode "{mode: 'MANUAL_CONTROL'}"   # 시작
ros2 service call /robot3/set_mode teamproject_interfaces/srv/SetMode "{mode: 'RESET'}"            # 종료
# 정지/복귀: EMERGENCY_STOP(래치 — RESUME/RESET만 해제) / RETURN_TO_CHARGER
# 관제 상태데이터: /robot3/nav_report (JSON 2Hz — 서버 중계용, 필드 명세는 회신_관제GUI_nav_report_0714.md 참고)
```

## 4. 검증 체크리스트 (순서대로)
1. `ping <Pi IP>` 0%손실·지연 안정 → `ros2 topic delay /scan` 0.0x (도메인 4 터미널에서)
2. `ros2 topic hz /scan`≈10, `/odom`≈20 — **이거 먼저 보장, 안 되면 출발 금지**
3. `ros2 node list | sort | uniq -d` 빈 출력(중복 0)
4. 카메라: `ros2 topic hz /robot3/camera/image_raw/compressed` ≈10fps
5. 마커: 로봇을 마커 정면 0.5~1m에 두고 `ros2 topic hz /detected_dock_pose` 수신 확인
6. 첫 언도크·도킹은 반드시 육안 감시 (옆에 다른 로봇 있으면 치워놓고)

## 5. 함정 모음 (로봇1·2가 흘린 피의 기록 — 꼭 읽기)
- **재기동 직후 15초 대기** 후 PATROL_START (DDS 디스커버리 전 명령은 증발)
- **로봇을 손으로 옮기면 반드시 위치 재주입 후 이동** (RViz 2D Pose Estimate 또는 도크에 놓고 시작) — 안 하면 지도가 통째로 어긋나 금지존 관통함
- **ros2 param set은 재기동하면 증발** — 검증된 값은 launch에 박제
- 임시로 띄운 프로세스(카메라 센더 등)는 재기동 킬에 안 죽고 살아남아 **이중송신→WiFi 폭주→scan 기근** 유발 — 임시 조치는 반드시 회수
- 도킹 SEARCH가 마커를 "발견→소실" 반복하면: ①마커 기울음/높이(중심 16cm) ②카메라 fps(10 권장) ③탐색속도(0.06) 순서로 점검
- 도킹이 삐뚤게 서면: wall_yaw_offset_deg에 실측 비틀림각 기입 (라이다 장착각 문제)
- 주차 횡위치 조정은 **마커를 물리로 옮기는 것**이 정답 (마커가 곧 도크 기준점)
- pgrep/pkill 자기매칭 주의: kill은 PID로, 계수는 `ps -eo args | awk '/패턴/ && !/awk/'`
- 여러 조 동시 사망/"어제는 됐는데" = 코드 무죄, WiFi/환경부터
