# docking — 정밀 도킹·이벤트 융합·안전 감시 노드

PC에서 실행되는 인지·도킹 계층입니다. 실제 기동 스크립트(launch/CLEAN_START)가 띄우는
최종 노드만 담았습니다. Pi에서 도는 도킹 실행기는 [`../pi/`](../pi/)에 있습니다.

## 도킹 체인
| 파일 | 역할 |
|---|---|
| `aruco_dock_detector.py` | ArUco 마커 pose 추정 (IPPE 모호성 폴백 포함) |
| `dock_detector_manager.py` | 도킹 시퀀스 중 검출기 on/off 관리 |
| `calib_marker_mappose.py` | 마커 map 좌표 캘리브 도구 |

## 이벤트·안전
| 파일 | 역할 |
|---|---|
| `safety_dispatch_fusion.py` | AI 감지(bbox)+라이다 융합 → 파견 판단, 위치 블랙리스트 |
| `keepout_intrusion_watch.py` | 금지구역 침범 자동 감시 (침범 0건 검증에 사용) |
| `id2_follow_detector.py` | 이벤트 대상 추종 검출 |

## 영상·캘리브레이션
| 파일 | 역할 |
|---|---|
| `udp_camera_bridge.py` | UDP JPEG chunk 수신 → ROS 이미지 (WiFi 포화 대책) |
| `publish_camera_info.py` | 캘리브 결과(ost.yaml) 발행 — fx/cx가 도킹 정밀도를 좌우 |
| `markers_map.yaml` | 마커 map 좌표 정의 |
