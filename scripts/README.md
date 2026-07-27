# scripts — 운영 스크립트

중복 노드가 서로 목표를 취소해 주행이 붕괴한 사고 이후, 기동·정지는 전부 스크립트로 통일했습니다.
구조는 공통 본체 하나 + 로봇별 얇은 래퍼입니다.

| 파일 | 역할 |
|---|---|
| `CLEAN_START_COMMON.sh` | 공통 본체 — 이전 인스턴스 정리 → Nav2 active 확인 → 카메라 순차 게이트. 같은 PC 두 로봇 병행 시 도메인 스코프 킬 |
| `CLEAN_START.sh` | 1호기 래퍼 (도메인 97) |
| `CLEAN_START_R2.sh` | 2호기 래퍼 (도메인 88) |
| `CLEAN_START_R3.sh` | 3호기 래퍼 (도메인 4) |
| `KILL_ALL.sh` | 전체 정지 (pkill 자기매칭 사고 방지 — 킬은 PID 기준) |
| `ROBOT_CONFIG.sh` | 로봇별 주소·도메인·포트 설정 단일 소스 |

충전 교대는 초기에 별도 릴레이 프로세스로 중계했으나, 1ms 경합으로 두 대가 동시 출동하는
문제를 겪고 최종 구조에서는 서버가 직접 지시하는 방식으로 단순화했습니다
(상세: [../docs/troubleshooting.md](../docs/troubleshooting.md) 5번).
