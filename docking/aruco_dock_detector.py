#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco 도킹 검출 노드 (PC에서 실행)
=====================================
opennav_docking(SimpleChargingDock, use_external_detection_pose:true)에
"마커 pose"를 먹여주는 눈 역할. 이 노드가 도킹을 직접 하지 않는다.

★ 왜 PC에서 도는데 대역폭 괜찮은가:
   raw(/image_raw)가 아니라 compressed(JPEG) 토픽을 구독한다.
   320x240 JPEG는 프레임당 ~5~15KB → WiFi 부담 적음. raw 원격구독 금지 철칙 유지.

흐름: /robot1/camera/image_raw/compressed (JPEG)
      + /robot1/camera/camera_info (K, D)
      → cv2.aruco.detectMarkers → solvePnP(IPPE_SQUARE, 평면 사각)
      → PoseStamped(카메라 광학프레임) 를 /detected_dock_pose 로 발행

OpenCV 4.6 구 API 기준 (PC·Pi 둘 다 4.6.0 확인됨). 신 API 섞지 말 것.
"""

import math
import os
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped


def rotmat_to_quat(R):
    """3x3 회전행렬 → (x,y,z,w) 쿼터니언. scipy 의존성 없이 직접 계산."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return x, y, z, w


class ArucoDockDetector(Node):
    def __init__(self):
        super().__init__('aruco_dock_detector')

        # ── 파라미터 ──
        self.declare_parameter('image_topic', '/robot1/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/robot1/camera/camera_info')
        self.declare_parameter('output_topic', '/detected_dock_pose')
        self.declare_parameter('marker_id', 42)           # 도킹 마커 ID (ID0은 남이 캘리브용으로 사용 → 42로 회피)
        self.declare_parameter('marker_length', 0.12)      # 마커 변 길이(m) — 실제 인쇄 크기(12cm)로 맞춤
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('detect_rate', 10.0)        # Hz 상한(스로틀)
        self.declare_parameter('optical_frame', '')        # 비면 camera_info의 frame_id 사용
        # ★CALIB_BAKE(7/9): 캘리브 원본(ost.yaml)을 시작 시 직접 읽어 K/D를 굽는다.
        #   왜: 기존엔 camera_info 토픽에만 의존 → publish_camera_info가 죽으면 조용히
        #   근사K(fx554)로 되돌아가고, 마커거리가 2.29배 어긋나 도킹 기하가 붕괴한다.
        #   토픽이 오면 그 값이 덮어쓴다(동일값). 파일 없으면 근사K 폴백.
        self.declare_parameter('calib_yaml', os.path.expanduser('~/team_ws/calib/ost.yaml'))
        # camera_info도 calib_yaml도 없을 때 쓸 근사 내부파라미터. FOV로 fx=fy 추정,
        # cx/cy는 이미지 중앙. ★7/9 실측 hfov=28.3°(기존 기본 60°는 2.29배 오차 유발).
        self.declare_parameter('approx_hfov_deg', 28.32)
        # ★AMBIGUITY(7/9): 평면마커 자세추정의 두 해 모호성 처리.
        #   IPPE_SQUARE 는 항상 해 2개를 낸다. 재투영오차 비 e_best/e_second 가 1에 가까우면
        #   둘을 구분할 수 없다(= 마커가 정면에 가깝다 = 원근 단서 부족). 화각이 좁을수록 심함(우리 28°).
        #   ★관찰: 모호한 상황은 곧 '마커가 정면'이므로, 참 법선은 시선방향(마커→카메라)에 가깝다.
        #   → 모호하면 법선을 시선방향으로 대체(frontal fallback). 명확하면 추정 법선을 그대로 신뢰.
        #   이 처리를 안 하면 정면 마커에서 법선이 90° 뒤집혀 접근점이 옆으로 튄다(실주행 hdg+84° 사례).
        self.declare_parameter('ambiguity_ratio', 0.6)     # e_best/e_second 가 이 값 초과면 '모호'
        self.declare_parameter('frontal_fallback', True)   # 모호 시 법선=시선방향으로 대체

        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_len = float(self.get_parameter('marker_length').value)
        self.detect_rate = float(self.get_parameter('detect_rate').value)
        self.optical_frame_override = self.get_parameter('optical_frame').value

        # ── ArUco 사전/파라미터 (4.6 구 API) ──
        dict_name = self.get_parameter('dictionary').value
        dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # 코너 서브픽셀 정제 → pose 안정화(문서 4단계 권장)
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # 마커 프레임 3D 코너(중심 원점, z=0). detectMarkers 코너 순서와 일치:
        # [top-left, top-right, bottom-right, bottom-left]
        s = self.marker_len / 2.0
        self.obj_pts = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float32)

        # ── 카메라 내부 파라미터(캘리브 전엔 None) ──
        self.K = None
        self.D = None
        self.approx_K = False       # 근사K 사용 중이면 True(정밀도킹 전 캘리브 경고용)
        self.info_frame = 'camera'
        self.load_calib_yaml(self.get_parameter('calib_yaml').value)

        # ── 스로틀 상태 ──
        self.min_period = 1.0 / self.detect_rate if self.detect_rate > 0 else 0.0
        self.last_stamp = None

        # ── I/O ──
        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.create_subscription(CameraInfo, self.info_topic,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.image_topic,
                                 self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"aruco_dock_detector 시작: img='{self.image_topic}' "
            f"info='{self.info_topic}' → out='{self.out_topic}', "
            f"marker id={self.marker_id} len={self.marker_len}m dict={dict_name}")
        self.warned_no_info = False

    @staticmethod
    def frontal_rotation(tvec):
        """★모호할 때 쓰는 '정면 가정' 회전행렬.
        마커 +Z(법선)를 시선방향(마커→카메라 원점)에 정렬시킨다.
        나머지 축은 카메라 상방(-Y)을 참고해 정규직교로 채운다.
        (모호 = 마커가 정면 = 참 법선이 시선방향에 가깝다 → 이 근사가 안전)"""
        t = np.asarray(tvec, dtype=np.float64).reshape(3)
        n = -t / (np.linalg.norm(t) + 1e-12)        # 마커면 법선(카메라 쪽을 향함)
        up = np.array([0.0, -1.0, 0.0])             # 카메라 광학프레임에서 위쪽
        x = np.cross(up, n)
        nx = np.linalg.norm(x)
        if nx < 1e-6:                                # 시선이 상방과 평행한 퇴화 케이스
            x = np.array([1.0, 0.0, 0.0])
            x = x - n * float(x @ n)
            nx = np.linalg.norm(x)
        x = x / nx
        y = np.cross(n, x)
        return np.column_stack((x, y, n))            # x×y = n (오른손계)

    def load_calib_yaml(self, path):
        """★CALIB_BAKE: ost.yaml(cameracalibrator 산출물)에서 K/D를 직접 적재.
        실패해도 죽지 않음 — camera_info 토픽/근사K 폴백이 그대로 남는다."""
        if not path:
            return
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            self.get_logger().warn(f'calib_yaml 없음: {path} — camera_info 토픽/근사K에 의존')
            return
        try:
            with open(path) as f:
                ost = yaml.safe_load(f)
            self.K = np.array(ost['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            self.D = np.array(ost['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
            self.approx_K = False
            # hfov는 '이미지 반폭/fx'로 계산(cx가 아님 — cx는 광학중심 편차를 포함)
            half_w = float(ost['image_width']) / 2.0
            hfov = math.degrees(2.0 * math.atan(half_w / self.K[0, 0]))
            self.get_logger().info(
                f'★캘리브 적재: {path} fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f} (hfov≈{hfov:.1f}°)')
        except Exception as e:
            self.K = None
            self.D = None
            self.get_logger().error(f'calib_yaml 적재 실패({e}) — camera_info 토픽/근사K 폴백')

    def on_camera_info(self, msg: CameraInfo):
        # 캘리브 안 됐으면 K가 0이거나 width=0으로 옴 → 무효 취급(근사K로 대체)
        if msg.k[0] <= 0.0 or msg.width == 0:
            if msg.header.frame_id:
                self.info_frame = msg.header.frame_id
            return
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self.approx_K = False
        if msg.header.frame_id:
            self.info_frame = msg.header.frame_id

    def build_approx_K(self, w, h):
        """camera_info 없을 때 FOV로 근사 내부파라미터 생성(정밀도킹 전 임시)."""
        hfov = math.radians(self.get_parameter('approx_hfov_deg').value)
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        self.K = np.array([[fx, 0, w / 2.0],
                           [0, fx, h / 2.0],
                           [0, 0, 1.0]], dtype=np.float64)
        self.D = np.zeros((5, 1), dtype=np.float64)
        self.approx_K = True
        self.get_logger().warn(
            f"근사K 사용중(camera_info 빈값): fx={fx:.1f} {w}x{h} hfov="
            f"{self.get_parameter('approx_hfov_deg').value}° — 거리값 부정확, 정밀도킹 전 캘리브 필수",
            once=True)

    def on_image(self, msg: CompressedImage):
        # 스로틀 (헤더 스탬프 기준)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and (stamp - self.last_stamp) < self.min_period:
            return
        self.last_stamp = stamp

        # JPEG 디코드 → 그레이
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.get_logger().warn("이미지 디코드 실패", throttle_duration_sec=5.0)
            return

        # camera_info 유효값 없으면 이미지 크기로 근사K 생성
        if self.K is None:
            h, w = gray.shape[:2]
            self.build_approx_K(w, h)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return
        ids = ids.flatten()

        # 목표 마커만 사용
        idxs = np.where(ids == self.marker_id)[0]
        if len(idxs) == 0:
            return
        img_pts = corners[idxs[0]].reshape(4, 2).astype(np.float32)

        # ★평면 사각 PnP — 두 해를 모두 받아 모호성을 판정한다(solvePnPGeneric).
        n_sol, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            self.obj_pts, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n_sol < 1:
            return

        # 재투영오차 오름차순 정렬 → 0번이 best
        order = sorted(range(n_sol), key=lambda i: float(errs[i]))
        rvec, tvec = rvecs[order[0]], tvecs[order[0]]
        R, _ = cv2.Rodrigues(rvec)

        ambiguous = False
        if n_sol >= 2:
            e1, e2 = float(errs[order[0]]), float(errs[order[1]])
            ratio = (e1 / e2) if e2 > 1e-9 else 1.0
            ambiguous = ratio > float(self.get_parameter('ambiguity_ratio').value)
            if ambiguous and self.get_parameter('frontal_fallback').value:
                R = self.frontal_rotation(tvec)
                self.get_logger().info(
                    f'법선 모호(e1/e2={ratio:.2f}) → 시선방향으로 대체(정면 가정)',
                    throttle_duration_sec=2.0)

        qx, qy, qz, qw = rotmat_to_quat(R)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.optical_frame_override or self.info_frame
        out.pose.position.x = float(tvec[0][0])
        out.pose.position.y = float(tvec[1][0])
        out.pose.position.z = float(tvec[2][0])
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ArucoDockDetector()
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
