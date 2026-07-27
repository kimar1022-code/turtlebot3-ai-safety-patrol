#!/usr/bin/env python3
# 좋은 주행 한 바퀴를 /amcl_pose로 녹화 → 0.3m 간격 점들을 csv로 저장.
# 사용: (주행 시작 직전) python3 record_path.py  → 한 바퀴 후 Ctrl+C
#  결과: ~/team_ws/maps/tools/recorded_path.csv  (x,y,yaw)
import rclpy, math, csv, os
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
OUT=os.path.expanduser('~/team_ws/maps/tools/recorded_path.csv')
STEP=0.3
class Rec(Node):
    def __init__(s):
        super().__init__('path_recorder')
        s.last=None; s.f=open(OUT,'w'); s.w=csv.writer(s.f); s.w.writerow(['x','y','yaw']); s.n=0
        s.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',s.cb,10)
        s.get_logger().info(f'녹화 시작 → {OUT} (0.3m 간격, Ctrl+C로 종료)')
    def cb(s,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        if s.last is None or math.hypot(p.x-s.last[0],p.y-s.last[1])>=STEP:
            yaw=2*math.atan2(q.z,q.w)
            s.w.writerow([round(p.x,4),round(p.y,4),round(yaw,4)]); s.f.flush()
            s.last=(p.x,p.y); s.n+=1
            s.get_logger().info(f'  점{s.n}: ({p.x:.2f},{p.y:.2f})')
def main():
    rclpy.init(); n=Rec()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.f.close(); n.get_logger().info(f'종료. 총 {n.n}점 저장')
if __name__=='__main__': main()
