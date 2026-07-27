#!/usr/bin/env python3
# 기존 test14 7개 wp를 RViz 마커로 표시 (구체+번호+경로선). frame=map, latched.
# RViz에서: Add → By topic → /test14_markers/MarkerArray 추가하면 보임.
import rclpy, yaml, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
CFG=os.path.expanduser('~/team_ws/src/teamproject_navigation/config/waypoints.yaml')
class Show(Node):
    def __init__(s):
        super().__init__('wp_marker_show')
        q=QoSProfile(depth=1); q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        s.pub=s.create_publisher(MarkerArray,'/test14_markers',q)
        d=yaml.safe_load(open(CFG)); s.wps=d['patrol_waypoints']
        s.timer=s.create_timer(1.0,s.pub_once); s.done=False
    def pub_once(s):
        arr=MarkerArray()
        line=Marker(); line.header.frame_id='map'; line.ns='route'; line.id=200
        line.type=Marker.LINE_STRIP; line.action=Marker.ADD; line.scale.x=0.03
        line.color.r=0.1; line.color.g=0.4; line.color.b=1.0; line.color.a=0.8
        line.pose.orientation.w=1.0
        for i,w in enumerate(s.wps):
            sp=Marker(); sp.header.frame_id='map'; sp.ns='wp'; sp.id=i
            sp.type=Marker.SPHERE; sp.action=Marker.ADD
            sp.pose.position.x=float(w['x']); sp.pose.position.y=float(w['y']); sp.pose.orientation.w=1.0
            sp.scale.x=sp.scale.y=sp.scale.z=0.12
            sp.color.r=0.1; sp.color.g=1.0; sp.color.b=0.1; sp.color.a=0.9
            arr.markers.append(sp)
            tx=Marker(); tx.header.frame_id='map'; tx.ns='label'; tx.id=100+i
            tx.type=Marker.TEXT_VIEW_FACING; tx.action=Marker.ADD; tx.text=w['name']
            tx.pose.position.x=float(w['x']); tx.pose.position.y=float(w['y']); tx.pose.position.z=0.25
            tx.pose.orientation.w=1.0; tx.scale.z=0.18
            tx.color.r=tx.color.g=tx.color.b=1.0; tx.color.a=1.0
            arr.markers.append(tx)
            p=Point(); p.x=float(w['x']); p.y=float(w['y']); line.points.append(p)
        # 루프 닫기
        p0=Point(); p0.x=float(s.wps[0]['x']); p0.y=float(s.wps[0]['y']); line.points.append(p0)
        arr.markers.append(line)
        s.pub.publish(arr)
        if not s.done:
            s.get_logger().info(f'{len(s.wps)}개 wp 마커 발행 (/test14_markers). RViz에 MarkerArray 추가하세요.')
            s.done=True
def main():
    rclpy.init(); n=Show()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
if __name__=='__main__': main()
