#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dock_detector_manager.py — PC 헬퍼: 순찰 상태가 DOCKING일 때만 ArUco 검출노드 자동 ON/OFF
==========================================================================================
왜? 도킹(pi_dock의 마커 SEARCH/ALIGN/REVERSE)엔 /detected_dock_pose 가 필요 → 검출노드 ON.
    반면 순찰 중엔 카메라 compressed 스트림을 계속 당기면 WiFi 포화 → AMCL 드리프트 → 검출노드 OFF 유지.
    (7/2 오전 통합 실패의 주범이 '순찰 내내 검출노드 ON'이었음)

동작: patrol_commander의 /robot1/state('STATE|reason|index') 구독 →
      상태가 active_state(기본 DOCKING)이면 검출노드(aruco_dock_detector.py) subprocess 시작,
      벗어나면 종료. 게이트 OFF면 DOCKING 상태 자체가 안 생겨 → 이 헬퍼는 아무것도 안 함(무해).

실행(PC): ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
          python3 ~/team_ws/aruco_docking/dock_detector_manager.py
"""
import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_DETECTOR = os.path.expanduser('~/team_ws/aruco_docking/aruco_dock_detector.py')


class DockDetectorManager(Node):
    def __init__(self):
        super().__init__('dock_detector_manager')
        self.declare_parameter('active_state', 'DOCKING')     # 이 상태일 때만 검출 ON
        self.declare_parameter('detector_path', DEFAULT_DETECTOR)
        # ★NAMESPACE(7/7): 멀티로봇 대응 — /robot1 하드코딩 제거, robot_id로 상태토픽 구성
        self.declare_parameter('robot_id', 1)
        rid = self.get_parameter('robot_id').value
        state_topic = f'/robot{rid}/state'
        self.proc = None
        self.create_subscription(String, state_topic, self.on_state, 10)
        self.get_logger().info(
            f"dock_detector_manager: {state_topic} == '{self.get_parameter('active_state').value}' "
            f"일 때만 검출노드 ON (그 외 OFF)")

    def _running(self):
        return self.proc is not None and self.proc.poll() is None

    def on_state(self, msg):
        state = (msg.data or '').split('|')[0].strip()
        want = (state == self.get_parameter('active_state').value)
        if want and not self._running():
            self.start_detector()
        elif not want and self._running():
            self.stop_detector(reason=f'상태 {state}')

    def start_detector(self):
        path = self.get_parameter('detector_path').value
        if not os.path.exists(path):
            self.get_logger().error(f'검출노드 파일 없음: {path}')
            return
        self.get_logger().info('DOCKING 진입 → ArUco 검출노드 시작')
        # 프로세스그룹으로 띄워 깔끔히 종료(자식까지). 현재 env(도메인/RMW) 상속.
        self.proc = subprocess.Popen(['python3', path], preexec_fn=os.setsid)

    def stop_detector(self, reason=''):
        if not self._running():
            self.proc = None
            return
        self.get_logger().info(f'DOCKING 벗어남({reason}) → ArUco 검출노드 종료')
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.proc = None

    def destroy_node(self):
        self.stop_detector(reason='매니저 종료')
        super().destroy_node()


def main():
    rclpy.init()
    node = DockDetectorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
