#!/usr/bin/env python3
# =============================================================================
# publish_camera_info.py — 캘리브 결과(ost.yaml)를 /robot1/camera/camera_info 로
#   static 발행 (UDP 카메라 파이프라인엔 camera_info 발행자가 없어서 직접 채워줌).
#
# 배경: 카메라를 UDP 이중송신으로 바꾸면서 camera_info 발행이 딸려 빠짐 → 검출기가
#   근사K(fx≈554, hfov 60°)로 동작 → 도킹 왼쪽 치우침 + 사진 안 가운데.
#   cameracalibrator 로 캘리브 → SAVE(/tmp/calibrationdata.tar.gz) → 이 스크립트로 주입.
#
# 검출기(aruco_dock_detector.py)는 /robot1/camera/camera_info 를 구독하므로,
#   이 토픽에 제대로 된 K/D 를 실으면 자동으로 실제값 사용(approx_K=False). 코드수정 0.
#
# 사용:
#   # tar.gz 에서 바로 (SAVE 직후)
#   python3 publish_camera_info.py --tar /tmp/calibrationdata.tar.gz
#   # 또는 풀어둔 ost.yaml 지정
#   python3 publish_camera_info.py --yaml /tmp/ost.yaml
#
# 확인: ros2 topic echo /robot1/camera/camera_info --once
#       → k[0]=fx, k[2]=cx, k[4]=fy, k[5]=cy 가 캘리브값인지.
# =============================================================================
import argparse, io, math, sys, tarfile
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo


def load_ost(tar_path=None, yaml_path=None):
    """cameracalibrator 의 ost.yaml 을 dict 로 읽는다."""
    if yaml_path:
        with open(yaml_path) as f:
            return yaml.safe_load(f)
    # tar.gz 안에서 ost.yaml 찾기
    with tarfile.open(tar_path, 'r:*') as tf:
        member = next((m for m in tf.getmembers() if m.name.endswith('ost.yaml')), None)
        if member is None:
            names = [m.name for m in tf.getmembers()]
            sys.exit(f"[에러] tar 안에 ost.yaml 없음. 내용: {names}")
        return yaml.safe_load(io.BytesIO(tf.extractfile(member).read()))


def build_camera_info(ost, frame_id):
    ci = CameraInfo()
    ci.width  = int(ost['image_width'])
    ci.height = int(ost['image_height'])
    ci.distortion_model = ost.get('distortion_model', 'plumb_bob')
    ci.k = [float(x) for x in ost['camera_matrix']['data']]
    ci.d = [float(x) for x in ost['distortion_coefficients']['data']]
    ci.r = [float(x) for x in ost['rectification_matrix']['data']]
    ci.p = [float(x) for x in ost['projection_matrix']['data']]
    ci.header.frame_id = frame_id
    return ci


class CamInfoPub(Node):
    def __init__(self, ci, topic):
        super().__init__('camera_info_static_pub')
        # transient_local = 늦게 켜는 구독자(검출기)도 마지막값 받음(latched 효과)
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.ci = ci
        self.pub = self.create_publisher(CameraInfo, topic, qos)
        self.timer = self.create_timer(1.0, self.tick)   # 1Hz 재발행(항상 최신 스탬프)
        self.tick()
        self.get_logger().info(
            f"camera_info 발행 시작 → {topic}  {ci.width}x{ci.height} "
            f"fx={ci.k[0]:.2f} fy={ci.k[4]:.2f} cx={ci.k[2]:.2f} cy={ci.k[5]:.2f}")

    def tick(self):
        self.ci.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.ci)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tar',  default='/tmp/calibrationdata.tar.gz',
                    help='cameracalibrator SAVE 결과 tar.gz')
    ap.add_argument('--yaml', default=None, help='풀어둔 ost.yaml 직접 지정')
    ap.add_argument('--topic', default='/robot1/camera/camera_info')
    ap.add_argument('--frame-id', default='camera',
                    help='브릿지/검출기 기본 프레임과 일치해야 함')
    args = ap.parse_args()

    ost = load_ost(tar_path=args.tar, yaml_path=args.yaml)
    ci = build_camera_info(ost, args.frame_id)

    # fusion 이 쓸 실제 화각 계산해 알려줌 (fx 로부터)
    fx, cx = ci.k[0], ci.k[2]
    hfov = math.degrees(2.0 * math.atan((ci.width / 2.0) / fx))
    print("=" * 64)
    print(f"[캘리브 결과]  {ci.width}x{ci.height}")
    print(f"  fx={ci.k[0]:.2f}  fy={ci.k[4]:.2f}  cx={cx:.2f}  cy={ci.k[5]:.2f}")
    print(f"  D={['%.4f' % v for v in ci.d]}")
    print(f"  → 실제 수평화각 hfov = {hfov:.2f}°  (근사값 62.2°와 비교)")
    print(f"  → 중심 편차: cx-{ci.width/2:.0f}={cx - ci.width/2:+.1f}px "
          f"(양수=광학중심이 오른쪽 → 검출은 왼쪽으로 치우쳐 보임)")
    print("  fusion 반영:")
    print(f"    ros2 param set /safety_dispatch_fusion hfov_deg {hfov:.2f}")
    print("=" * 64)

    rclpy.init()
    node = CamInfoPub(ci, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
