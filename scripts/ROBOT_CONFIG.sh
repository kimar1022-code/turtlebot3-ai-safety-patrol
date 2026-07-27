#!/usr/bin/env bash
# =============================================================================
# ROBOT_CONFIG.sh — 로봇별 설정 (도메인, Pi IP, PC IP, 네임스페이스)
# 소스: source ~/team_ws/ROBOT_CONFIG.sh <robot_id>
# =============================================================================

ROBOT_ID="${1:-1}"   # 기본값 로봇1

case "$ROBOT_ID" in
  1)
    export ROS_DOMAIN_ID=97
    export ROBOT_PI=codelab@192.168.40.101
    export ROBOT_IP=192.168.40.101
    export ROBOT_PC_IP=192.168.40.7
    export ROBOT_NAMESPACE=/robot1
    export ROBOT_CAM_ENABLED=1    # 카메라 ON (5fps)
    echo "✅ 로봇1 설정: 도메인97, Pi .101, 카메라 ON"
    ;;
  2)
    export ROS_DOMAIN_ID=88
    export ROBOT_PI=codelab@192.168.40.102
    export ROBOT_IP=192.168.40.102
    export ROBOT_PC_IP=192.168.40.7
    export ROBOT_NAMESPACE=/robot2
    export ROBOT_CAM_ENABLED=0    # 카메라 OFF (서버 YOLO만 사용)
    echo "✅ 로봇2 설정: 도메인88, Pi .102, 카메라 OFF (서버 YOLO)"
    ;;
  3)
    export ROS_DOMAIN_ID=4
    export ROBOT_PI=codelab@192.168.40.103
    export ROBOT_IP=192.168.40.103
    export ROBOT_PC_IP=192.168.40.7
    export ROBOT_NAMESPACE=/robot3
    export ROBOT_CAM_ENABLED=0    # 카메라 OFF (서버 YOLO만 사용)
    echo "✅ 로봇3 설정: 도메인4, Pi .103, 카메라 OFF (서버 YOLO)"
    ;;
  *)
    echo "❌ 로봇ID 오류: 1, 2, 3만 지원"
    return 1
    ;;
esac

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds_unicast.xml
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source "$HOME/team_ws/install/setup.bash"
