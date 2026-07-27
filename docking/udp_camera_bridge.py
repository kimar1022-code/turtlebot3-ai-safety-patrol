#!/usr/bin/env python3
"""
UDP GlobalCam(JPEG-over-UDP) → ROS CompressedImage 브릿지 (PC 측).

Pi의 `turtlebot_udp_camera_sender.py`가 보내는 UDP JPEG 스트림을 받아
`/robot1/camera/image_raw/compressed` 로 재발행한다.
→ 서버(YOLO)는 UDP를 직접 받고, PC(도킹)는 이 브릿지로 ROS 토픽을 얻어
  같은 카메라 한 통신을 둘이 나눠 쓰는 구조.
→ ArUco 검출노드(aruco_dock_detector.py)는 수정 없이 그대로 이 토픽을 구독한다.
  (camera_info 없으면 검출노드가 이미지 크기로 근사K 자동 생성)

GlobalCam 와이어포맷 (sender와 반드시 동일):
  헤더 34B  !4sBHIQHHIHHB =
    MAGIC(b'GCM1') VERSION header_size frame_seq timestamp_ns
    width height jpeg_size total_chunks chunk_index frame_id_len
  뒤에  frame_id(utf-8, frame_id_len 바이트) + JPEG 조각(chunk_size 이하)
  한 프레임 = total_chunks개 조각 → frame_seq로 묶어 chunk_index 순서로 재조립.

실행(PC):
  python3 ~/team_ws/aruco_docking/udp_camera_bridge.py --ros-args -p port:=5007

주의: throttle 때문에 매 프레임 신선한 stamp(=수신시각)를 찍는다.
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

MAGIC = b"GCM1"
HEADER_FORMAT = "!4sBHIQHHIHHB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)   # 34


class _Frame:
    """재조립 중인 한 프레임(조각 버퍼)."""
    __slots__ = ("total", "chunks", "count")

    def __init__(self, total: int):
        self.total = total
        self.chunks = [None] * total
        self.count = 0


class UdpCameraBridge(Node):
    def __init__(self):
        super().__init__("udp_camera_bridge")

        # ── 파라미터 ──
        self.declare_parameter("port", 5007)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("output_topic", "/robot1/camera/image_raw/compressed")
        self.declare_parameter("max_pending", 8)   # 동시 재조립 프레임 상한(늦은/유실 프레임 청소)

        self.port = int(self.get_parameter("port").value)
        bind_host = self.get_parameter("bind_host").value
        out_topic = self.get_parameter("output_topic").value
        self.max_pending = int(self.get_parameter("max_pending").value)

        self.pub = self.create_publisher(CompressedImage, out_topic, qos_profile_sensor_data)

        # ── UDP 소켓 ──
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.bind((bind_host, self.port))
        self.sock.settimeout(1.0)

        # ── 상태 ──
        self.frames = {}          # frame_seq -> _Frame
        self.frame_id = "camera"
        self.rx_packets = 0
        self.rx_frames = 0
        self.bad_magic = 0
        self._stop = False

        self.get_logger().info(
            f"UDP 카메라 브릿지 시작: {bind_host}:{self.port} → '{out_topic}' "
            f"(GlobalCam JPEG 재조립)")

        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self._stats)

    def _stats(self):
        self.get_logger().info(
            f"rx_packets={self.rx_packets} rx_frames={self.rx_frames} "
            f"pending={len(self.frames)} bad_magic={self.bad_magic}")

    def _rx_loop(self):
        while not self._stop and rclpy.ok():
            try:
                pkt, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) < HEADER_SIZE:
                continue

            (magic, _ver, _hsize, seq, _ts, _w, _h,
             _jsize, total, cidx, fidlen) = struct.unpack(HEADER_FORMAT, pkt[:HEADER_SIZE])
            if magic != MAGIC:
                self.bad_magic += 1
                continue
            self.rx_packets += 1

            if fidlen:
                try:
                    self.frame_id = pkt[HEADER_SIZE:HEADER_SIZE + fidlen].decode("utf-8")
                except UnicodeDecodeError:
                    pass
            payload = pkt[HEADER_SIZE + fidlen:]

            if total <= 0 or cidx >= total:
                continue

            fr = self.frames.get(seq)
            if fr is None:
                fr = _Frame(total)
                self.frames[seq] = fr
                # 오래된 미완성 프레임 청소(유실된 조각 때문에 영영 안 차는 것 방지)
                if len(self.frames) > self.max_pending:
                    for old in sorted(self.frames)[:-self.max_pending]:
                        del self.frames[old]

            if fr.chunks[cidx] is None:
                fr.chunks[cidx] = payload
                fr.count += 1

            if fr.count == fr.total:
                data = b"".join(fr.chunks)
                self.frames.pop(seq, None)
                self._publish(data)

    def _publish(self, jpeg: bytes):
        msg = CompressedImage()
        # ★ 매 프레임 신선한 스탬프(수신시각). 검출노드 throttle이 stamp 기준이라 필수.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub.publish(msg)
        self.rx_frames += 1

    def destroy_node(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UdpCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
