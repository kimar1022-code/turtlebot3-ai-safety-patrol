#!/usr/bin/env python3
# compressed→raw 변환 (캘리브레이터가 raw만 구독해서 필요. QoS: 구독=best_effort(브릿지와 일치), 발행=reliable(캘리브레이터와 일치))
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage


class C2R(Node):
    def __init__(self):
        super().__init__('compressed_to_raw')
        self.pub = self.create_publisher(Image, '/robot1/camera/image_raw', 5)
        self.create_subscription(CompressedImage, '/robot1/camera/image_raw/compressed',
                                 self.cb, qos_profile_sensor_data)
        self.n = 0

    def cb(self, m):
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        out = Image()
        out.header = m.header
        out.height, out.width = img.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = img.tobytes()
        self.pub.publish(out)
        self.n += 1
        if self.n == 1:
            self.get_logger().info(f'첫 프레임 변환 OK ({out.width}x{out.height})')


rclpy.init()
rclpy.spin(C2R())
