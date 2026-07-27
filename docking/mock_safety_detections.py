#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
가짜 safety detections 퍼블리셔 (safety_dispatch_fusion 테스트용)
================================================================
서버 YOLO 없이 `/robotN/safety/detections`(std_msgs/String, JSON)에
가짜 검출을 흘려서 융합 노드의 파싱→방위각→라이다융합→파견 체인을 검증한다.

사용 예:
  # 화면 정중앙(cx=160)에 fire, conf 0.9, 2Hz
  python3 mock_safety_detections.py
  # 왼쪽(cx=80)에 fall, 5프레임만 발행하고 종료
  python3 mock_safety_detections.py --ros-args -p class_name:=fall -p cx:=80.0 -p count:=5
  # 스키마 변형 테스트: bbox만 주는 미니멀 payload
  python3 mock_safety_detections.py --ros-args -p style:=minimal

style 파라미터:
  rich    = ★서버 확정 스키마 v1.0(2026-07-07) 그대로:
            schema_version + created_at(ISO-8601 Z) + frame_stamp(촬영시각 float 초,
            서버에 요청중인 필드 — 1순위 파싱 경로 테스트용) + camera_id
            + source_topic + detections[{class, confidence, bbox_xyxy, center_px, face_result}]
  minimal = class/score/bbox 만(다른 키 이름) — 관대한 파서 검증용
"""

import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockSafetyDetections(Node):
    def __init__(self):
        super().__init__('mock_safety_detections')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('topic', '')            # 비면 /robot{N}/safety/detections
        self.declare_parameter('rate', 2.0)            # 발행 Hz
        self.declare_parameter('class_name', 'fire')   # fire|fall|head|no_helmet (확정 4종)
        self.declare_parameter('confidence', 0.9)
        self.declare_parameter('cx', 160.0)            # bbox 중심 x픽셀 (320폭 중앙=160)
        self.declare_parameter('cy', 120.0)
        self.declare_parameter('box_w', 60.0)          # bbox 크기(중심 기준 생성)
        self.declare_parameter('box_h', 80.0)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('include_stamp', True)  # created_at 포함 여부
        self.declare_parameter('include_frame_stamp', True)  # frame_stamp(촬영시각) 포함 여부
        self.declare_parameter('style', 'rich')        # rich(확정 스키마) | minimal
        self.declare_parameter('count', 0)             # 0=무한, N=그만큼 발행 후 정지

        rid = int(self.get_parameter('robot_id').value)
        topic = (self.get_parameter('topic').value
                 or f'/robot{rid}/safety/detections')
        self.pub = self.create_publisher(String, topic, 10)
        self._n = 0
        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f"mock 발행 시작: '{topic}' {rate}Hz "
            f"class={self.get_parameter('class_name').value} "
            f"conf={self.get_parameter('confidence').value} "
            f"cx={self.get_parameter('cx').value} "
            f"style={self.get_parameter('style').value}")

    def _tick(self):
        g = lambda n: self.get_parameter(n).value
        cnt = int(g('count'))
        if cnt > 0 and self._n >= cnt:
            return   # 발행 끝(노드는 살려둠 — Ctrl+C 로 종료)

        cx, cy = float(g('cx')), float(g('cy'))
        bw, bh = float(g('box_w')), float(g('box_h'))
        bbox = [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2]
        rid = int(g('robot_id'))

        if g('style') == 'minimal':
            # 다른 키 이름(label/score/bbox)만 있는 최소 payload — 관대한 파서 검증
            payload = {'detections': [{
                'label': g('class_name'),
                'score': float(g('confidence')),
                'bbox': bbox,
            }]}
        else:   # rich: ★서버 확정 스키마 v1.0 그대로
            payload = {
                'schema_version': '1.0',
                'camera_id': f'robot{rid}',
                'source_topic': f'/robot{rid}/live/image',
                'detections': [{
                    'class': g('class_name'),
                    'confidence': float(g('confidence')),
                    'bbox_xyxy': bbox,
                    'center_px': [cx, cy],
                    'face_result': None,
                }],
            }
            if bool(g('include_stamp')):
                # created_at: ISO-8601 UTC, 'Z' 접미(서버 표기와 동일. ms 정밀도)
                payload['created_at'] = datetime.now(timezone.utc).isoformat(
                    timespec='milliseconds').replace('+00:00', 'Z')
            if bool(g('include_frame_stamp')):
                # ★frame_stamp: 촬영시각 float 초(서버 요청중 필드) — 융합 노드가
                #   created_at 보다 1순위로 파싱하는 경로 테스트용
                payload['frame_stamp'] = datetime.now(timezone.utc).timestamp()

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)
        self._n += 1
        self.get_logger().info(f'발행 #{self._n}: {msg.data}',
                               throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = MockSafetyDetections()
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
