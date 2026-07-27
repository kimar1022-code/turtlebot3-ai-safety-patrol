#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure_dock_range.py — pre-dock 지점에서 servo_standoff 실측값을 읽는다.
=========================================================================
왜 필요한가:
  pi_dock_executor 의 마커거리 파라미터(servo_standoff 등)는 '검출기가 뱉는 거리' 단위다.
  7/9 캘리브 적용으로 fx 가 554.3(근사) → 1268.3(실측) 으로 바뀌면서 그 단위가 2.29배
  달라졌다. 따라서 옛 튜닝값(0.33)은 더 이상 유효하지 않고, 실측으로 다시 잡아야 한다.

무엇을 재는가:
  pi_dock_executor.on_marker() 와 완전히 동일한 정의:
      marker_range = ||tvec||   (카메라 광학원점 → 마커중심, 3D 직선거리)
  bearing = atan2(y, x)  (base 기준, 좌+)  — 참고용

사용법:
  1) 로봇을 '프리도킹(pre-dock)' 자리에 놓고 마커(ID42)가 화면에 보이게 한다.
  2) python3 ~/team_ws/aruco_docking/measure_dock_range.py
  3) 값이 안정되면(표준편차 작음) 출력된 '중앙값'을 servo_standoff 에 넣는다.

주의: 검출기(aruco_dock_detector.py)가 실측K로 떠 있어야 한다.
      로그에 '★캘리브 적재' 가 보여야 하고 '근사K 사용중' 경고가 없어야 한다.
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

# 캘리브(ost.yaml) 값 — 마커 중심의 화면 픽셀 u 계산용
FX = 1268.3021
CX = 362.31794
IMG_W = 640


class DockRangeMeasure(Node):
    def __init__(self):
        super().__init__('measure_dock_range')
        self.declare_parameter('window', 30)     # 통계 낼 샘플 수
        self.win = int(self.get_parameter('window').value)
        self.rngs = []
        self.bears = []
        self.us = []
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_pose, 10)
        self.get_logger().info(
            '/detected_dock_pose 대기중... 마커(ID42)가 카메라에 보여야 합니다. Ctrl+C 로 종료.')

    def on_pose(self, msg):
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        rng = float(np.linalg.norm(tvec))           # ★pi_dock 의 marker_range 와 동일 정의
        p = R_BASE_OPT @ tvec
        bearing = math.degrees(math.atan2(float(p[1]), float(p[0])))
        # ★마커 중심의 화면 픽셀 u = cx + fx·(X/Z)  (광학프레임 X=우, Z=전방)
        #   u=320(화면중심) 이면 '마커가 화면 정중앙', u=cx(362.3) 이면 'bearing=0(광학축 위)'.
        u = CX + FX * (float(tvec[0]) / float(tvec[2])) if abs(tvec[2]) > 1e-6 else float('nan')

        self.rngs.append(rng)
        self.bears.append(bearing)
        self.us.append(u)
        if len(self.rngs) > self.win:
            self.rngs.pop(0); self.bears.pop(0); self.us.pop(0)

        if len(self.rngs) < 5:
            self.get_logger().info(f'수집중... range={rng:.3f}m u={u:.0f}px')
            return

        med = statistics.median(self.rngs)
        sd = statistics.pstdev(self.rngs)
        bmed = statistics.median(self.bears)
        umed = statistics.median(self.us)
        self.get_logger().info(
            f'range={med:.3f}m (σ{sd*1000:.1f}mm)  bearing={bmed:+.2f}°  '
            f'화면u={umed:.0f}px (중심320, 광학중심{CX:.0f})  '
            f'화면중심까지 {umed-320:+.0f}px',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = DockRangeMeasure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if len(node.rngs) >= 5:
            med = statistics.median(node.rngs)
            sd = statistics.pstdev(node.rngs)
            print('\n' + '=' * 56)
            print(f'  최종 range 중앙값 = {med:.3f} m  (표준편차 {sd*1000:.1f}mm)')
            print(f'  → servo_standoff 에 넣을 값: {med:.3f}')
            print('=' * 56)
        else:
            print('\n[경고] 샘플 부족 — 마커가 검출되지 않았습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
