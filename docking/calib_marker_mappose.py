#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calib_marker_mappose.py — 마커 맵좌표 자동 캘리브 (로봇이 알려진 pose에 있을 때)
==============================================================================
로봇을 정확히 (robot_x, robot_y, robot_yaw)에 놓고 실행하면,
현재 마커 검출(/detected_dock_pose, ID42)로부터 마커의 맵좌표를 역산해
markers_map.yaml 에 저장한다.

★핵심: aruco_localizer 와 '똑같은' 카메라변환(R_BASE_OPT, cam_x/y/z)을 쓴다.
   그래서 카메라 오프셋이 부정확해도, localizer가 역으로 풀 때 오차가 상쇄되어
   로봇 실제위치를 정확히 복원한다(self-consistent).

실행(로봇을 0,0,0에 놓고):
  python3 calib_marker_mappose.py --ros-args -p marker_id:=42 -p robot_x:=0.0 -p robot_y:=0.0 -p robot_yaw_deg:=0.0
"""
import math
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

R_BASE_OPT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def quat_to_rotmat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


class CalibMarker(Node):
    def __init__(self):
        super().__init__('calib_marker_mappose')
        self.declare_parameter('marker_topic', '/detected_dock_pose')
        self.declare_parameter('markers_file', os.path.expanduser('~/team_ws/aruco_docking/markers_map.yaml'))
        self.declare_parameter('marker_id', 42)
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('robot_yaw_deg', 0.0)
        self.declare_parameter('cam_x', 0.05)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.113)
        self.declare_parameter('samples', 20)   # 이만큼 평균

        g = lambda n: self.get_parameter(n).value
        self.T_base_cam = np.eye(4)
        self.T_base_cam[:3, :3] = R_BASE_OPT
        self.T_base_cam[:3, 3] = [g('cam_x'), g('cam_y'), g('cam_z')]
        # 로봇 맵pose T_map_base
        ry = math.radians(g('robot_yaw_deg'))
        c, s = math.cos(ry), math.sin(ry)
        self.T_map_base = np.array([[c, -s, 0, g('robot_x')],
                                    [s,  c, 0, g('robot_y')],
                                    [0,  0, 1, 0.0],
                                    [0,  0, 0, 1.0]])
        self.samples = []
        self.need = int(g('samples'))
        self.create_subscription(PoseStamped, g('marker_topic'), self.on_marker, 10)
        self.get_logger().info(
            f"캘리브 시작: 로봇=({g('robot_x')},{g('robot_y')},{g('robot_yaw_deg')}°), "
            f"마커 ID{g('marker_id')} — {self.need}샘플 수집중...")

    def on_marker(self, msg):
        if len(self.samples) >= self.need:
            return
        q = msg.pose.orientation
        tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = quat_to_rotmat(q.x, q.y, q.z, q.w)
        T_cam_marker[:3, 3] = tvec
        T_base_marker = self.T_base_cam @ T_cam_marker
        T_map_marker = self.T_map_base @ T_base_marker
        mx, my, mz = T_map_marker[0, 3], T_map_marker[1, 3], T_map_marker[2, 3]
        # 마커 법선(+z_marker) 맵방향 → yaw
        nx, ny = T_map_marker[0, 2], T_map_marker[1, 2]
        myaw = math.atan2(ny, nx)
        self.samples.append((mx, my, mz, myaw))
        if len(self.samples) == self.need:
            self.save()

    def save(self):
        arr = np.array(self.samples)
        mx = float(np.median(arr[:, 0])); my = float(np.median(arr[:, 1])); mz = float(np.median(arr[:, 2]))
        myaw = math.atan2(float(np.median(np.sin(arr[:, 3]))), float(np.median(np.cos(arr[:, 3]))))
        mid = int(self.get_parameter('marker_id').value)
        path = self.get_parameter('markers_file').value
        data = {}
        if os.path.exists(path):
            data = yaml.safe_load(open(path)) or {}
        data.setdefault('markers', {})
        data['markers'][mid] = {'x': round(mx, 4), 'y': round(my, 4),
                                'yaw_deg': round(math.degrees(myaw), 2), 'z': round(mz, 4)}
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, allow_unicode=True)
        self.get_logger().info(
            f"✅ ID{mid} 맵좌표 저장: x={mx:.3f} y={my:.3f} yaw={math.degrees(myaw):.1f}° z={mz:.3f} → {path}")
        self.get_logger().info("완료. Ctrl-C로 종료하고 aruco_localizer 실행하세요.")


def main():
    rclpy.init()
    node = CalibMarker()
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
