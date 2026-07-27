#!/usr/bin/env python3
# waypoints_dense.yaml(또는 clicked_wp.yaml 순서)을 번호+연결선으로 표시 → /dense_preview
import rclpy, yaml, os, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
SRC=os.path.expanduser('~/clicked_wp.yaml')   # 클릭 순서 그대로
class S(Node):
    def __init__(s):
        super().__init__('dense_preview')
        q=QoSProfile(depth=1); q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        s.pub=s.create_publisher(MarkerArray,'/dense_preview',q)
        d=yaml.safe_load(open(SRC)); s.pts=[(float(x),float(y)) for x,y in d['clicked_points']]
        s.create_timer(1.0,s.go); s.done=False
    def go(s):
        arr=MarkerArray()
        ln=Marker(); ln.header.frame_id='map'; ln.ns='order'; ln.id=0; ln.type=Marker.LINE_STRIP
        ln.action=Marker.ADD; ln.scale.x=0.025; ln.color.r=1.0; ln.color.g=0.5; ln.color.b=0.0; ln.color.a=0.9; ln.pose.orientation.w=1.0
        for i,(x,y) in enumerate(s.pts):
            p=Point(); p.x=x; p.y=y; ln.points.append(p)
            t=Marker(); t.header.frame_id='map'; t.ns='num'; t.id=600+i; t.type=Marker.TEXT_VIEW_FACING
            t.action=Marker.ADD; t.text=str(i+1); t.pose.position.x=x; t.pose.position.y=y; t.pose.position.z=0.22
            t.pose.orientation.w=1.0; t.scale.z=0.16; t.color.r=1.0; t.color.g=1.0; t.color.b=0.0; t.color.a=1.0
            arr.markers.append(t)
        arr.markers.append(ln)
        s.pub.publish(arr)
        if not s.done: s.get_logger().info(f'{len(s.pts)}점 순서선 발행 → /dense_preview (RViz에 MarkerArray 추가)'); s.done=True
def main():
    rclpy.init(); n=S()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
if __name__=='__main__': main()
