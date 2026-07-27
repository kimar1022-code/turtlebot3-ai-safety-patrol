#!/usr/bin/env python3
"""★MANUAL_GATE — 관제 수동조작 cmd_vel 게이트 (멀티로봇 네임스페이스 대응, 2026-07-07)

왜 필요한가:
  - 인터페이스 계약: 서버(관제 GUI)는 `/robotN/cmd_vel`에 발행하고,
    로봇은 MANUAL_CONTROL 상태에서만 반영해야 한다.
  - 기존엔 이 게이트가 없어서 ①/robotN/cmd_vel 구독자가 아예 없고(수동조작 무반응)
    ②상태 무관하게 루트 /cmd_vel에 직접 쏘면 자율주행과 충돌하는 구멍이 있었다.

동작:
  - 구독: 'cmd_vel' (상대경로 → launch namespace로 /robotN/cmd_vel)  [Twist 또는 TwistStamped, 파라미터]
  - 구독: 'state'   (상대경로 → /robotN/state, patrol_commander FSM 상태)
  - 발행: '/cmd_vel' (루트 = 로봇 베이스. 본 프로젝트 베이스는 TwistStamped)
  - 게이트: FSM 상태가 MANUAL_CONTROL일 때만 통과. 그 외 상태에선 무시(+warn 스로틀).
  - 안전장치:
    * 데드맨: MANUAL_CONTROL 중 입력이 cmd_timeout(기본 0.5s) 끊기면 0속도 발행(GUI/WiFi 끊김 폭주 방지)
    * 상태 이탈: MANUAL_CONTROL을 벗어나는 순간 0속도 1회 발행(잔류 속도 제거)

멀티로봇: launch에서 namespace=/robotN로 띄우면 로봇별 /robotN/cmd_vel이 자동 분리된다.
🔴 실주행 미검증 — 서버팀에 입력 타입(Twist인지 TwistStamped인지) 확인 후 input_stamped 맞출 것.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String


class ManualCmdGate(Node):
    def __init__(self):
        super().__init__('manual_cmd_gate')
        self.declare_parameter('robot_id', 1)
        # 서버 GUI가 보내는 타입. 일반 GUI/teleop 기본은 Twist라 False가 기본값.
        # 서버가 TwistStamped를 보내면 -p input_stamped:=true
        self.declare_parameter('input_stamped', False)
        self.declare_parameter('cmd_timeout', 0.5)   # 데드맨(초)
        self.declare_parameter('active_state', 'MANUAL_CONTROL')

        self.active_state = self.get_parameter('active_state').value
        self.cur_state = ''           # patrol FSM 상태('STATE|reason|wp'의 STATE 부분)
        self.last_cmd_time = None     # 마지막 수동명령 수신 시각
        self.was_active = False       # 직전 틱에 MANUAL이었는지(이탈 시 0속도 1회용)

        # 베이스는 TwistStamped 구독(본 프로젝트 확정: teleop도 stamped 필요)
        self.out_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        if self.get_parameter('input_stamped').value:
            self.create_subscription(TwistStamped, 'cmd_vel', self._on_stamped, 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_twist, 10)

        # /robotN/state 는 transient_local(라치) 발행 — 맞춰서 늦게 떠도 현재 상태 수신
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, 'state', self._on_state, state_qos)

        self.create_timer(0.1, self._watchdog)   # 10Hz 데드맨/이탈 감시
        self.get_logger().info(
            f"manual_cmd_gate up | robot_id={self.get_parameter('robot_id').value} "
            f"| 입력={'TwistStamped' if self.get_parameter('input_stamped').value else 'Twist'} "
            f"| {self.active_state} 상태에서만 /cmd_vel 통과")

    # ---------- 콜백 ----------
    def _on_state(self, msg):
        self.cur_state = msg.data.split('|')[0]

    def _on_twist(self, msg: Twist):
        self._forward(msg)

    def _on_stamped(self, msg: TwistStamped):
        self._forward(msg.twist)

    def _forward(self, tw: Twist):
        if self.cur_state != self.active_state:
            # MANUAL 아닌데 수동명령이 옴 — 반영 안 함(서버측 발행 규율 위반 감지용 로그)
            self.get_logger().warn(
                f'수동 cmd_vel 무시: 현재 상태 {self.cur_state or "?"} != {self.active_state}',
                throttle_duration_sec=2.0)
            return
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = tw
        self.out_pub.publish(out)
        self.last_cmd_time = self.get_clock().now()

    # ---------- 안전장치 ----------
    def _publish_zero(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        self.out_pub.publish(out)

    def _watchdog(self):
        active = (self.cur_state == self.active_state)
        if self.was_active and not active:
            # MANUAL 이탈 순간: 잔류 속도 제거 (이후엔 발행 안 함 — 자율주행 cmd_vel과 충돌 금지)
            self._publish_zero()
            self.last_cmd_time = None
        elif active and self.last_cmd_time is not None:
            dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if dt > self.get_parameter('cmd_timeout').value:
                self._publish_zero()   # 데드맨: 입력 끊김 → 정지 유지
        self.was_active = active


def main(args=None):
    rclpy.init(args=args)
    node = ManualCmdGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node.destroy_node()


if __name__ == '__main__':
    main()
