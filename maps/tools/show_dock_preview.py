#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
show_dock_preview.py — 현재 활성 순찰 waypoints + pre-dock + 충전독을 RViz 마커로 표시
  · 회색 선/구+번호   : 순찰 경로(dense, waypoints.yaml)
  · 청록 큰 화살표+글자 : wp28 = PRE-DOCK(도킹독 40cm 앞), yaw 방향 표시
  · 빨강 큐브+글자      : DOCK(충전독, 0,0)
발행 토픽: /wp_preview (MarkerArray, frame=map, latched)
RViz: Add → MarkerArray → Topic=/wp_preview
실행: python3 ~/team_ws/maps/tools/show_dock_preview.py
"""
import rclpy, yaml, os, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

WP = os.path.expanduser('~/team_ws/install/teamproject_navigation/share/teamproject_navigation/config/waypoints.yaml')

def yaw_of(w):
    return 2*math.atan2(w.get('yaw_z',0.0), w.get('yaw_w',1.0))

class Prev(Node):
    def __init__(s):
        super().__init__('show_dock_preview')
        q = QoSProfile(depth=1); q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        s.pub = s.create_publisher(MarkerArray, '/wp_preview', q)
        d = yaml.safe_load(open(WP))
        s.wps = d['patrol_waypoints']
        s.dock = d.get('charging_station', {'x':0.0,'y':0.0})
        s.timer = s.create_timer(1.0, s.tick); s.n = 0
        s.get_logger().info(f'{len(s.wps)}개 wp + pre-dock + dock 발행 → /wp_preview (RViz에 MarkerArray 추가)')

    def tick(s):
        arr = MarkerArray(); last = len(s.wps)-1
        # 경로 선
        ln = Marker(); ln.header.frame_id='map'; ln.ns='path'; ln.id=0
        ln.type=Marker.LINE_STRIP; ln.action=Marker.ADD; ln.scale.x=0.02
        ln.color.r=0.6; ln.color.g=0.6; ln.color.b=0.6; ln.color.a=0.7; ln.pose.orientation.w=1.0
        from geometry_msgs.msg import Point
        for w in s.wps:
            p=Point(); p.x=float(w['x']); p.y=float(w['y']); p.z=0.02; ln.points.append(p)
        arr.markers.append(ln)
        # 각 wp: 구 + 번호 + (pre-dock은 특별표시)
        for i,w in enumerate(s.wps):
            x,y = float(w['x']), float(w['y']); is_pre = (i==last)
            sp=Marker(); sp.header.frame_id='map'; sp.ns='wp'; sp.id=i
            sp.type=Marker.SPHERE; sp.action=Marker.ADD
            sp.pose.position.x=x; sp.pose.position.y=y; sp.pose.position.z=0.05; sp.pose.orientation.w=1.0
            if is_pre:
                sp.scale.x=sp.scale.y=sp.scale.z=0.14; sp.color.r=0.0; sp.color.g=0.9; sp.color.b=0.9; sp.color.a=1.0
            else:
                sp.scale.x=sp.scale.y=sp.scale.z=0.07; sp.color.r=0.3; sp.color.g=0.5; sp.color.b=1.0; sp.color.a=0.9
            arr.markers.append(sp)
            tx=Marker(); tx.header.frame_id='map'; tx.ns='num'; tx.id=100+i
            tx.type=Marker.TEXT_VIEW_FACING; tx.action=Marker.ADD
            tx.pose.position.x=x; tx.pose.position.y=y; tx.pose.position.z=0.22; tx.pose.orientation.w=1.0
            tx.scale.z=0.13; tx.color.r=1.0; tx.color.g=1.0; tx.color.b=1.0; tx.color.a=1.0
            tx.text = f'PRE-DOCK(wp{i})' if is_pre else f'wp{i}'
            arr.markers.append(tx)
            # yaw 화살표
            aw=Marker(); aw.header.frame_id='map'; aw.ns='yaw'; aw.id=200+i
            aw.type=Marker.ARROW; aw.action=Marker.ADD
            aw.pose.position.x=x; aw.pose.position.y=y; aw.pose.position.z=0.05
            th=yaw_of(w); aw.pose.orientation.z=math.sin(th/2); aw.pose.orientation.w=math.cos(th/2)
            aw.scale.x=0.20 if is_pre else 0.12; aw.scale.y=0.03; aw.scale.z=0.03
            aw.color.r=0.0 if is_pre else 1.0; aw.color.g=0.9; aw.color.b=0.9 if is_pre else 0.0; aw.color.a=1.0
            arr.markers.append(aw)
        # 충전독(0,0) 빨강 큐브 + 글자
        dk=Marker(); dk.header.frame_id='map'; dk.ns='dock'; dk.id=0
        dk.type=Marker.CUBE; dk.action=Marker.ADD
        dk.pose.position.x=float(s.dock['x']); dk.pose.position.y=float(s.dock['y']); dk.pose.position.z=0.04; dk.pose.orientation.w=1.0
        dk.scale.x=dk.scale.y=0.12; dk.scale.z=0.08; dk.color.r=1.0; dk.color.g=0.1; dk.color.b=0.1; dk.color.a=1.0
        arr.markers.append(dk)
        dt=Marker(); dt.header.frame_id='map'; dt.ns='dock'; dt.id=1
        dt.type=Marker.TEXT_VIEW_FACING; dt.action=Marker.ADD
        dt.pose.position.x=float(s.dock['x']); dt.pose.position.y=float(s.dock['y']); dt.pose.position.z=0.20; dt.pose.orientation.w=1.0
        dt.scale.z=0.13; dt.color.r=1.0; dt.color.g=0.3; dt.color.b=0.3; dt.color.a=1.0; dt.text='DOCK(0,0)'
        arr.markers.append(dt)
        s.pub.publish(arr)
        s.n+=1
        if s.n==3: s.get_logger().info('발행 중(latched). RViz MarkerArray Topic=/wp_preview')

def main():
    rclpy.init(); rclpy.spin(Prev())

if __name__=='__main__':
    main()
