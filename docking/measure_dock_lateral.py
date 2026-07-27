#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure_dock_lateral.py — lateral_target 실측값을 읽는다 (2026-07-10)
=========================================================================
왜 필요한가:
  pi_dock_executor 의 횡오차 lat_off 는 marker_range(=||tvec||) 와 **같은 카메라 단위**다.
      lat_off = (R_bm.T @ (-p))[0],   p = R_BASE_OPT @ tvec
  7/9 캘리브 적용(fx 554.3 근사 → 1268.3 실측)으로 이 단위가 ×2.288 바뀌었고,
  servo_standoff(0.33→0.734) / srv_pos_tol / srv_range_tol 은 재스케일했지만
  **lateral_target(0.075) 과 center_lat_tol(0.01) 은 옛 스케일 그대로 남았다.**
  → 도킹이 매번 같은 쪽(왼쪽)으로 ~1cm 치우치는 계통오차의 유력 원인.

무엇을 재는가:
  로봇을 '물리적으로 정확히 중앙'인 자리에 놓으면 참 횡오차 = 0 이다.
  그때 검출기가 뱉는 lat_off 가 곧 상수 편향(카메라 장착 오프셋 + 마커 부착 오차)이고,
  그 값이 그대로 lateral_target 에 들어갈 값이다. (servo_standoff 0.734 를 잡은 것과 같은 방법)

사용법:
  1) 로봇을 '프리도킹(pre-dock)' 자리에, 눈으로 보아 충전기 정중앙에 오도록 자를 대고 놓는다.
     (좌우 치우침이 0이어야 한다. 이 전제가 틀리면 측정값도 그만큼 틀린다.)
  2) python3 ~/team_ws/aruco_docking/measure_dock_lateral.py
  3) 값이 안정되면(σ 작음) 출력된 '중앙값'을 lateral_target 에 넣는다.

주의: 검출기(aruco_dock_detector.py)가 실측K로 떠 있어야 한다.
      로그에 '★캘리브 적재' 가 보이고 '근사K 사용중' 경고가 없어야 한다.
      캘리브를 다시 하면 이 값도 다시 재야 한다.
"""
import math
import statistics
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# pi_dock_executor 와 동일: 광학프레임(z앞,x우,y아래) → base(x앞,y좌,z위)
R_BASE_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def quat_to_rotmat(x, y, z, w):
    """pi_dock_executor 와 동일한 정의."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class DockLateralMeasure(Node):
    def __init__(self):
        super().__init__('measure_dock_lateral')
        self.declare_parameter('window', 40)
        self.win = int(self.get_parameter('window').value)
        self.lats = []
        self.rngs = []
        self.bears = []
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_pose, 10)
        self.get_logger().info(
            '/detected_dock_pose 대기중... 로봇을 충전기 정중앙(프리도킹)에 놓으세요. Ctrl+C 로 종료.')

    def on_pose(self, msg):
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        # ── pi_dock_executor.on_marker() 와 글자 그대로 동일 ──
        p = R_BASE_OPT @ tvec
        R_bm = R_BASE_OPT @ quat_to_rotmat(q.x, q.y, q.z, q.w)
        p_robot_in_marker = R_bm.T @ (-p)
        lat_off = float(p_robot_in_marker[0])
        rng = float(np.linalg.norm(tvec))
        bearing = math.degrees(math.atan2(float(p[1]), float(p[0])))

        self.lats.append(lat_off)
        self.rngs.append(rng)
        self.bears.append(bearing)
        for a in (self.lats, self.rngs, self.bears):
            if len(a) > self.win:
                a.pop(0)

        if len(self.lats) < 5:
            self.get_logger().info(f'수집중... lat={lat_off * 100:+.2f}cm range={rng:.3f}m')
            return

        lmed = statistics.median(self.lats)
        lsd = statistics.pstdev(self.lats)
        self.get_logger().info(
            f'lat_off={lmed * 100:+.2f}cm (σ{lsd * 1000:.1f}mm)  '
            f'range={statistics.median(self.rngs):.3f}m  '
            f'bearing={statistics.median(self.bears):+.2f}°',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = DockLateralMeasure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if len(node.lats) >= 5:
            lmed = statistics.median(node.lats)
            lsd = statistics.pstdev(node.lats)
            rmed = statistics.median(node.rngs)
            print('\n' + '=' * 64)
            print(f'  lat_off 중앙값 = {lmed:+.4f}  (표준편차 {lsd * 1000:.1f}mm)')
            print(f'  같은 자리 range = {rmed:.3f}  (servo_standoff 와 비교용)')
            print()
            print(f'  → lateral_target 에 넣을 값: {lmed:.4f}')
            print(f'     (현재값 0.075 와 크게 다르면, 7/9 캘리브 재스케일 누락이 확정)')
            print('=' * 64)
        else:
            print('\n[경고] 샘플 부족 — 마커가 검출되지 않았습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
