#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keepout_intrusion_watch.py — 금지존 침범 현행범 체포기 (2026-07-10)
=========================================================================
왜 필요한가:
  "금지구역을 돌진했다"는 사실은 눈으로 보이는데, 로그만으로는 **누가 밟았는지** 못 가린다.
  용의자가 셋이다:
    ① 랩 서보 주행      — move_guard_scope='event' 라 가드 범위 밖
    ② Nav2 ESCAPE 폴백  — Spin/DriveOnHeading, 코스트맵을 안 봄
    ③ 진입/재개 직선 레그 — _seg_blocked 로 검사하지만 전역 코스트맵 미수신이면 통과

무엇을 하는가:
  /amcl_pose 를 받을 때마다 로봇 발자국(robot_radius)이 keepout 마스크 안에 들어갔는지 검사하고,
  들어간 순간의 **FSM 상태**(/robot1/state)와 좌표·침범깊이를 ERROR 로그로 남긴다.
  침범이 끝나면 지속시간과 최대 깊이를 요약한다.

사용법:
  python3 ~/team_ws/aruco_docking/keepout_intrusion_watch.py
  (CLEAN_START 와 무관하게 아무 때나 붙였다 뗄 수 있는 순수 관찰자 — 아무것도 발행하지 않는다)

읽는 파일: ~/team_ws/maps/keepout_mask.{yaml,pgm}
"""
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

MASK_YAML = os.path.expanduser('~/team_ws/maps/keepout_mask_newmap.yaml')  # ★실사용 마스크(map.yaml과 짝). keepout_mask.yaml 은 옛 맵용


def load_mask(yaml_path):
    """의존성 없이 yaml 최소 파싱 + PGM(P5) 로드."""
    import yaml
    meta = yaml.safe_load(open(yaml_path))
    pgm = meta['image']
    if not os.path.isabs(pgm):
        pgm = os.path.join(os.path.dirname(yaml_path), pgm)
    res = float(meta['resolution'])
    org = [float(v) for v in meta['origin']]
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))

    f = open(pgm, 'rb')
    assert f.readline().strip() == b'P5', 'PGM(P5) 아님'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = map(int, line.split())
    maxv = int(f.readline())
    data = f.read()
    return dict(w=w, h=h, maxv=maxv, data=data, res=res, org=org,
                negate=negate, occ=occ_th)


class KeepoutWatch(Node):
    def __init__(self):
        super().__init__('keepout_intrusion_watch')
        self.declare_parameter('robot_radius', 0.10)   # burger
        self.declare_parameter('samples', 8)           # 발자국 원주 샘플 수
        self.m = load_mask(MASK_YAML)
        self.state = '?'
        self.inside = False
        self.t0 = None
        self.max_depth = 0.0
        self.state_at_entry = '?'

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(String, '/robot1/state', self.on_state, 10)
        self.get_logger().info(
            f"금지존 감시 시작 — 마스크 {self.m['w']}x{self.m['h']} res={self.m['res']}. "
            f"침범 시 FSM 상태와 좌표를 ERROR 로 남깁니다.")

    def on_state(self, msg):
        self.state = (msg.data or '?').split('|')[0]

    def _keepout(self, x, y):
        m = self.m
        cx = int((x - m['org'][0]) / m['res'])
        cy = int((y - m['org'][1]) / m['res'])
        if not (0 <= cx < m['w'] and 0 <= cy < m['h']):
            return False
        px = m['data'][(m['h'] - 1 - cy) * m['w'] + cx]
        occ = px / m['maxv'] if m['negate'] else (m['maxv'] - px) / m['maxv']
        return occ >= m['occ']

    def _depth(self, x, y):
        """중심 + 발자국 원주 샘플 중 몇 개가 금지존인가 → 침범 깊이 추정."""
        r = float(self.get_parameter('robot_radius').value)
        n = int(self.get_parameter('samples').value)
        hits = 1 if self._keepout(x, y) else 0
        for i in range(n):
            a = 2.0 * math.pi * i / n
            if self._keepout(x + r * math.cos(a), y + r * math.sin(a)):
                hits += 1
        return hits, n + 1

    def on_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        hits, total = self._depth(x, y)
        now = self.get_clock().now().nanoseconds / 1e9
        frac = hits / total

        if hits > 0 and not self.inside:
            self.inside = True
            self.t0 = now
            self.max_depth = frac
            self.state_at_entry = self.state
            self.get_logger().error(
                f'★★금지존 침범 시작 — 상태={self.state}  map({x:+.3f},{y:+.3f})  '
                f'발자국 {hits}/{total} 셀 침범')
        elif hits > 0:
            self.max_depth = max(self.max_depth, frac)
            self.get_logger().error(
                f'★금지존 안 — 상태={self.state} map({x:+.3f},{y:+.3f}) {hits}/{total}',
                throttle_duration_sec=1.0)
        elif hits == 0 and self.inside:
            self.inside = False
            dur = now - self.t0 if self.t0 else 0.0
            self.get_logger().error(
                f'★★금지존 탈출 — 진입시 상태={self.state_at_entry}, 탈출시={self.state}, '
                f'지속 {dur:.1f}s, 최대 침범 {self.max_depth*100:.0f}%')
            self.t0 = None
            self.max_depth = 0.0


def main():
    rclpy.init()
    node = KeepoutWatch()
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
