#!/usr/bin/env python3
# RViz "Publish Point"로 클릭한 점들을 받아 저장.
# 클릭할 때마다 ~/clicked_wp.yaml 에 누적 + 마커로 빨간 점/번호 표시(/clicked_markers).
import rclpy, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
OUT=os.path.expanduser('~/clicked_wp.yaml')
class Rec(Node):
    def __init__(s):
        super().__init__('click_recorder')
        s.pts=[]
        s.create_subscription(PointStamped,'/clicked_point',s.cb,10)
        q=QoSProfile(depth=1); q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        s.mpub=s.create_publisher(MarkerArray,'/clicked_markers',q)
        open(OUT,'w').write('clicked_points:\n')
        s.get_logger().info(f'클릭 대기 중 → {OUT}  (RViz Publish Point로 찍으세요)')
    def cb(s,m):
        x,y=m.point.x,m.point.y; s.pts.append((x,y))
        with open(OUT,'a') as f: f.write(f'  - [{x:.4f}, {y:.4f}]\n')
        s.get_logger().info(f'  점 {len(s.pts)}: ({x:.2f}, {y:.2f})')
        arr=MarkerArray()
        for i,(px,py) in enumerate(s.pts):
            d=Marker(); d.header.frame_id='map'; d.ns='clk'; d.id=i; d.type=Marker.SPHERE; d.action=Marker.ADD
            d.pose.position.x=px; d.pose.position.y=py; d.pose.orientation.w=1.0
            d.scale.x=d.scale.y=d.scale.z=0.1; d.color.r=1.0; d.color.g=0.2; d.color.b=0.2; d.color.a=1.0
            arr.markers.append(d)
            t=Marker(); t.header.frame_id='map'; t.ns='clkn'; t.id=500+i; t.type=Marker.TEXT_VIEW_FACING; t.action=Marker.ADD
            t.text=str(i+1); t.pose.position.x=px; t.pose.position.y=py; t.pose.position.z=0.2; t.pose.orientation.w=1.0
            t.scale.z=0.15; t.color.r=t.color.g=t.color.b=1.0; t.color.a=1.0
            arr.markers.append(t)
        s.mpub.publish(arr)
def main():
    rclpy.init(); n=Rec()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
if __name__=='__main__': main()
