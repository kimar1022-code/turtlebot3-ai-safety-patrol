#!/usr/bin/env python3
# waypoints_dense.yaml 최종경로를 초록선+번호로 표시 → /dense_final
import rclpy, yaml, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
SRC=os.path.expanduser('~/team_ws/src/teamproject_navigation/config/waypoints_dense.yaml')
class S(Node):
    def __init__(s):
        super().__init__('dense_final')
        q=QoSProfile(depth=1); q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        s.pub=s.create_publisher(MarkerArray,'/dense_final',q)
        s.wps=yaml.safe_load(open(SRC))['patrol_waypoints']
        s.create_timer(1.0,s.go); s.done=False
    def go(s):
        arr=MarkerArray()
        ln=Marker(); ln.header.frame_id='map'; ln.ns='path'; ln.id=0; ln.type=Marker.LINE_STRIP
        ln.action=Marker.ADD; ln.scale.x=0.03; ln.color.g=1.0; ln.color.a=0.9; ln.pose.orientation.w=1.0
        for i,w in enumerate(s.wps):
            p=Point(); p.x=float(w['x']); p.y=float(w['y']); ln.points.append(p)
            t=Marker(); t.header.frame_id='map'; t.ns='n'; t.id=700+i; t.type=Marker.TEXT_VIEW_FACING
            t.action=Marker.ADD; t.text=str(i); t.pose.position.x=float(w['x']); t.pose.position.y=float(w['y']); t.pose.position.z=0.2
            t.pose.orientation.w=1.0; t.scale.z=0.13; t.color.g=1.0; t.color.b=0.3; t.color.a=1.0
            arr.markers.append(t)
        p0=Point(); p0.x=float(s.wps[0]['x']); p0.y=float(s.wps[0]['y']); ln.points.append(p0)  # 루프닫기
        arr.markers.append(ln); s.pub.publish(arr)
        if not s.done: s.get_logger().info(f'{len(s.wps)}점 최종경로 → /dense_final'); s.done=True
def main():
    rclpy.init(); n=S()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
if __name__=='__main__': main()
