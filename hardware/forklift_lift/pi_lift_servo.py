#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pi_lift_servo.py — Pi 로컬 지게차 리프트 서보 구동노드 (3호기, GPIO18, 상주)
============================================================================
서버가 수동조작할 때 리프트도 같이 수동조작되게 한다.

3호기는 수동조작 전용이다 — 카메라/라이다/Nav2/patrol_commander 전부 안 쓴다.
따라서 PC측 노드가 0개이고, 이 노드가 Pi에서 주행 릴레이 + 리프트 구동을 겸한다.

명령 경로:
  서버 ros_client.publish_manual_control()
      twist.linear.x  = 전후 주행
      twist.angular.z = 회전
      twist.linear.z  = +1.0 올림 / -1.0 내림 / 0.0 정지   ← 서버팀 추가분
    → /robot3/cmd_vel (TwistStamped, 도메인 4)
    → 이 노드
        ├─ linear.x/angular.z → /cmd_vel 로 릴레이 → turtlebot3_node (주행)
        └─ linear.z           → GPIO18 서보 PWM      (리프트)

왜 이 노드가 릴레이까지 하나?
  1·2호기는 manual_cmd_gate(PC)가 /robot3/cmd_vel→/cmd_vel 중계 + MANUAL 게이팅 +
  0.5s 데드맨을 담당한다. 그런데 그 게이트는 patrol_commander 가 발행하는 /robotN/state 를
  봐야 문을 연다. 3호기엔 라이다가 없어 Nav2/patrol_commander 가 안 돌므로 state 가 없고,
  게이트를 그대로 쓰면 문이 영원히 안 열린다. → 릴레이와 데드맨을 이 노드가 인수한다.
  덕분에 3호기는 PC 없이 Pi 단독으로 서버 수동조작이 가능하다.

왜 Pi 로컬? pi_dock_executor 와 같은 이유 — WiFi 지연으로 '정지'가 늦으면 리프트가
기구 한계까지 밀고 올라가고, 주행은 벽을 박는다. 모션과 정지는 전부 Pi 로컬에서.

왜 linear.z? 이미 흐르는 수동조작 TwistStamped 의 미사용 필드다. turtlebot3_node 는
linear.z 를 무시하므로 주행과 완전 무간섭이고, 같은 메시지 하나로 주행+리프트를 동시
조작할 수 있다. 서버는 필드 하나만 추가하면 된다.

동작:
  z > +deadzone → 목표 펄스폭을 us_up   방향으로 ramp_us_per_sec 로 증가
  z < -deadzone → 목표 펄스폭을 us_down 방향으로 감소
  그 외 / 명령 끊김(cmd_timeout) → 현 위치 유지(서보는 계속 홀드, 짐이 안 떨어짐)
  ★ 개루프다. 서보엔 위치 피드백이 없으므로 부팅 시 리프트를 '완전히 내린' 상태로 둘 것.
    첫 명령이 오기 전까지는 펄스를 아예 안 쏜다(부팅 트위치 방지 — 서보는 그동안 무력).

안전:
  us_down/us_up 이 소프트 리밋. 기구 하드스톱에 물리기 전 값으로 잡을 것(기본 1000~2000us는
  풀스트로크가 아닌 보수적 범위. 실측 후 넓혀라).
  램프만 하고 점프하지 않는다 → '천천히'가 기구적으로 보장된다.
  종료 시 펄스 정지(서보 무력화).

실행(Pi, 도메인 필수 — 3호기는 4):
  ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 ~/pi_lift_servo.py
"""
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float32


# ---------------------------------------------------------------------------
# PWM 백엔드 — 하드웨어 타이밍 우선. GPIO18은 Pi의 하드웨어 PWM0 핀이라
# pigpio/lgpio 를 쓰면 라이다+카메라 CPU 부하에도 펄스가 안 흔들린다.
# 소프트웨어 PWM(RPi.GPIO)으로 떨어지면 서보가 지지직거리므로 최후수단.
# ---------------------------------------------------------------------------
class _SysfsPWM:
    """SoC 하드웨어 PWM (/sys/class/pwm). CPU 부하와 무관하게 지터가 없다.

    전제: /boot/firmware/config.txt 에 `dtoverlay=pwm-2chan` + 재부팅.
          (Ubuntu on Pi 는 /boot 가 아니라 /boot/firmware 다)
          GPIO18→PWM0(채널 0), GPIO19→PWM1(채널 1).
    권한: sysfs 쓰기에 root 또는 udev 규칙이 필요할 수 있다.
    """
    PERIOD_NS = 20_000_000                      # 50Hz = 서보 표준

    CH = {18: 0, 19: 1}                         # BCM 핀 → PWM 채널

    def __init__(self, pin, chip='pwmchip0'):
        if pin not in self.CH:
            raise RuntimeError(f'GPIO{pin} 은 하드웨어 PWM 핀이 아니다(18 또는 19만 가능)')
        ch = self.CH[pin]
        base = f'/sys/class/pwm/{chip}'
        if not os.path.isdir(base):
            raise RuntimeError(f'{base} 없음 — dtoverlay=pwm-2chan 미적용 또는 재부팅 필요')
        self.path = f'{base}/pwm{ch}'
        if not os.path.isdir(self.path):
            with open(f'{base}/export', 'w') as f:
                f.write(str(ch))
            time.sleep(0.1)                     # udev 가 노드를 만들 때까지
        if not os.path.isdir(self.path):
            raise RuntimeError(f'채널 {ch} export 실패')
        self._set('period', self.PERIOD_NS)
        self._set('duty_cycle', 0)
        self._set('enable', 0)

    def _set(self, name, val):
        with open(f'{self.path}/{name}', 'w') as f:
            f.write(str(int(val)))

    def write(self, us):
        """us=None → 출력 정지(서보 무력화)."""
        if us is None:
            self._set('enable', 0)
            self._set('duty_cycle', 0)
        else:
            self._set('duty_cycle', int(us) * 1000)   # us → ns
            self._set('enable', 1)

    def close(self):
        try:
            self.write(None)
        except Exception:
            pass


class ServoBackend:
    """서보 펄스폭(us) 출력 백엔드. write(None) 이면 펄스 정지(서보 무력화)."""

    def __init__(self, pin, logger, use_hw_pwm=False):
        self.pin = pin
        self.log = logger
        self.kind = None
        self._h = None
        self._pi = None
        self._pwm = None
        self._last = None

        # ★기본은 lgpio. 하드웨어 PWM 은 use_hw_pwm:=true 로 명시 opt-in.
        #   이유(2026-07-20 실측): dtoverlay=pwm-2chan 을 인자 없이 넣으면 이 커널에서
        #   PWM0 이 GPIO18 로 안 물린다. 그런데 sysfs 는 정상적으로 존재하므로 초기화가
        #   '성공'하고, enable=1/duty=1250us 로 멀쩡히 보고하면서 실제 펄스는 엉뚱한 핀으로
        #   나간다. → 조용히 실패한다. 서보가 죽은 것으로 오진할 뻔했다.
        #   하드웨어 PWM 을 쓰려면 핀을 명시할 것:
        #     dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
        #   그리고 반드시 서보가 실제로 도는지 눈으로 확인한 뒤 기본값으로 승격할 것.
        if not use_hw_pwm:
            self.log.info('PWM 백엔드 선택: lgpio (하드웨어 PWM 은 use_hw_pwm:=true 로 활성화)')
        # 1순위 sysfs 하드웨어 PWM (SoC PWM 페리페럴 = 지터 0)
        #   전제: /boot/firmware/config.txt 에 dtoverlay=pwm-2chan (GPIO18→PWM0) + 재부팅
        #   ★lgpio 저자 경고(tx_servo 문서): "소프트웨어 타이밍 서보 펄스는 테스트용으로만
        #     써라. 타이밍 지터가 서보를 떨게 하고, 이로 인해 과열되고 수명이 단축될 수 있다."
        #     2026-07-20 SG90 과열 사고 이후 하드웨어 PWM 을 기본으로 승격.
        #   노드가 죽어도 마지막 duty 가 유지되므로 리프트가 갑자기 떨어지지 않는다.
        if use_hw_pwm:
            try:
                self._hw = _SysfsPWM(pin)
                self.kind = 'hwpwm'
                self.log.warn(
                    f'PWM 백엔드=하드웨어 PWM (sysfs {self._hw.path}). '
                    f'★sysfs 초기화 성공이 곧 GPIO{pin} 출력을 뜻하지 않는다 — '
                    f'서보가 실제로 도는지 반드시 눈으로 확인할 것')
                return
            except Exception as e:
                self.log.warn(f'하드웨어 PWM 사용불가({e}) → lgpio 로 폴백')

        # 2순위 lgpio (소프트웨어 타이밍 — 지터 있음, 서보 발열 주의)
        try:
            import lgpio
            self._lgpio = lgpio
            self._h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self._h, pin)
            self.kind = 'lgpio'
            self.log.warn('PWM 백엔드=lgpio (소프트웨어 타이밍). 지터로 서보가 떨 수 있고 '
                          '장시간 사용 시 발열/수명 저하. 하드웨어 PWM 권장.')
            return
        except Exception as e:
            self.log.warn(f'lgpio 사용불가: {e}')

        # 3순위 RPi.GPIO 소프트웨어 PWM (떨림 있음)
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT)
            self._pwm = GPIO.PWM(pin, 50)
            self._pwm.start(0)
            self.kind = 'rpigpio'
            self.log.warn('PWM 백엔드=RPi.GPIO 소프트웨어 PWM — 서보 떨림 가능. '
                          'pigpio 설치 권장: sudo apt install pigpio && sudo systemctl enable --now pigpiod')
            return
        except Exception as e:
            self.log.error(f'RPi.GPIO 사용불가: {e}')

        raise RuntimeError('사용 가능한 GPIO PWM 백엔드가 없다 (pigpio/lgpio/RPi.GPIO 전부 실패)')

    def write(self, us):
        """us=None → 펄스 정지. 같은 값이면 재발행 생략(lgpio/pigpio는 유지형 출력)."""
        # ★값이 바뀔 때만 발행한다. tx_servo 는 '한 번 부르면 계속 도는' 지속형이고,
        #   매 틱 부르면 PWM 큐에 쌓여 대부분의 명령이 사이클 경계에서 버려진다.
        if us == self._last:
            return
        self._last = us
        if self.kind == 'hwpwm':
            self._hw.write(us)
        elif self.kind == 'lgpio':
            if us is None:
                # ★lgpio 상류 버그: tx_servo(...,0) 은 '활성 레코드가 있을 때'만 정지를
                #   처리한다. 첫 정지로 레코드가 비활성화되면 tx 스레드가 ~20ms 안에
                #   free() 하고, 이후 정지 호출은 'bad PWM micros' 로 죽는다.
                #   → 예외를 삼키고, 라인은 직접 LOW 로 내린다.
                #   (tx_servo(0) 은 xWrite 를 안 해서 펄스 도중 멈추면 GPIO 가 HIGH 로 굳는다)
                try:
                    self._lgpio.tx_servo(self._h, self.pin, 0)
                except Exception:
                    pass                        # 이미 멈춰 있음 — 무해
                self._lgpio.gpio_write(self._h, self.pin, 0)
            else:
                self._lgpio.tx_servo(self._h, self.pin, int(us))
        elif self.kind == 'rpigpio':
            # 50Hz → 주기 20000us. duty% = us/20000*100
            self._pwm.ChangeDutyCycle(0.0 if us is None else (us / 200.0))

    def close(self):
        try:
            self.write(None)
        except Exception:
            pass
        try:
            if self.kind == 'hwpwm':
                self._hw.close()
            elif self.kind == 'lgpio':
                self._lgpio.gpiochip_close(self._h)
            elif self.kind == 'rpigpio':
                self._pwm.stop()
                self._GPIO.cleanup(self.pin)
        except Exception:
            pass


class LiftServo(Node):
    def __init__(self):
        super().__init__('pi_lift_servo')

        d = self.declare_parameter
        d('gpio_pin', 18)
        d('cmd_topic', '/robot3/cmd_vel')   # 서버가 쏘는 수동조작 토픽
        d('input_stamped', True)            # 서버는 TwistStamped 로 발행
        d('relay_drive', True)              # linear.x/angular.z 를 /cmd_vel 로 중계
        d('drive_topic', '/cmd_vel')        # turtlebot3_node 가 구독하는 토픽
        d('us_down', 500.0)                 # 리프트 최하단 펄스폭 (0도, sg90 테스트 기준)
        d('us_up', 1500.0)                  # 리프트 최상단 펄스폭 (90도, sg90 테스트 기준)
        d('ramp_us_per_sec', 100000.0)      # 사실상 즉시 점프 (sg90 SPACE 토글과 동일 느낌)
        d('rate_hz', 50.0)                  # 서보 갱신 주기(50Hz = 서보 표준)
        d('deadzone', 0.2)                  # |linear.z| 이하는 정지로 간주
        d('cmd_timeout', 0.5)               # 명령 끊기면 정지(게이트 데드맨과 동일)
        # --- 스톨/발열 방지 (2026-07-20 서보 과열 사고 후 추가) ---
        d('stall_timeout', 2.0)             # 한계에 물린 채 이 시간 넘게 밀면 물러남
        d('stall_backoff_us', 15.0)         # 물러나는 양
        d('idle_release_sec', 5.0)          # 최하단에서 이만큼 무명령이면 펄스 해제(0=비활성)
        d('bottom_tol_us', 20.0)            # '최하단에 있다' 판정 여유
        d('use_hw_pwm', False)              # ★기본 OFF — 아래 ServoBackend 주석 참조
        # ★현재 리프트 위치(us). 개루프라 노드는 위치를 알 수 없어 기본은 us_down(최하단)
        #   으로 가정한다. 리프트가 최하단이 아닌 상태에서 노드를 띄우면 첫 명령에
        #   가정 위치로 튀므로(짐 낙하 위험), 그럴 땐 실제 위치를 여기에 넣을 것.
        #   예: 리프트가 최상단(1000us)에 있는데 재시작 → -p start_us:=1000.0
        d('start_us', -1.0)                 # <0 이면 us_down 으로 가정

        g = lambda n: self.get_parameter(n).value
        self.pin = int(g('gpio_pin'))
        self.us_down = float(g('us_down'))
        self.us_up = float(g('us_up'))
        self.ramp = float(g('ramp_us_per_sec'))
        self.rate = float(g('rate_hz'))
        self.deadzone = float(g('deadzone'))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.stall_timeout = float(g('stall_timeout'))
        self.stall_backoff = float(g('stall_backoff_us'))
        self.idle_release = float(g('idle_release_sec'))
        self.bottom_tol = float(g('bottom_tol_us'))

        self.us_lo = min(self.us_down, self.us_up)
        self.us_hi = max(self.us_down, self.us_up)
        # 올림 방향의 부호 (us_up < us_down 인 역방향 장착도 지원)
        self.dir_up = 1.0 if self.us_up >= self.us_down else -1.0

        # 개루프: 첫 명령 전까지 펄스 미발행. 시작 위치는 start_us(미지정 시 최하단 가정).
        _start = float(g('start_us'))
        self.cur_us = _start if _start > 0 else self.us_down
        if _start > 0:
            self.cur_us = min(self.us_hi, max(self.us_lo, self.cur_us))
        self.armed = False
        self.cmd_z = 0.0
        self.last_cmd_t = None

        # 주행 릴레이 (manual_cmd_gate 대체분)
        self.relay = bool(g('relay_drive'))
        self.last_twist = None
        self.drive_zeroed = True            # 데드맨 0속도를 이미 쐈는가

        self._pin_since = None              # 한계에 물리기 시작한 시각
        self._backed_off = False            # 이번 물림에서 이미 물러났는가
        self._last_move_t = None            # 마지막으로 실제 이동한 시각
        self._released = False              # 최하단 유휴 해제 상태인가

        self.backend = ServoBackend(self.pin, self.get_logger(),
                                    use_hw_pwm=bool(g('use_hw_pwm')))

        topic = g('cmd_topic')
        if bool(g('input_stamped')):
            self.create_subscription(TwistStamped, topic, self._on_stamped, 10)
        else:
            self.create_subscription(Twist, topic, self._on_twist, 10)

        self.drive_pub = self.create_publisher(TwistStamped, g('drive_topic'), 10) \
            if self.relay else None

        # 관제/디버그용 현재 위치 0.0(최하단)~1.0(최상단)
        self.pos_pub = self.create_publisher(Float32, 'lift_position', 10)

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'리프트 서보 기동: GPIO{self.pin} 백엔드={self.backend.kind} '
            f'범위={self.us_down:.0f}~{self.us_up:.0f}us 램프={self.ramp:.0f}us/s '
            f'명령={topic}.linear.z  '
            f'주행릴레이={"ON → " + g("drive_topic") if self.relay else "OFF"}  '
            f'★부팅 시 리프트는 최하단이어야 함')

    # ---- 명령 수신 -------------------------------------------------------
    def _on_stamped(self, msg):
        self._set_cmd(msg.twist.linear.z, msg.twist)

    def _on_twist(self, msg):
        self._set_cmd(msg.linear.z, msg)

    def _set_cmd(self, z, twist=None):
        self.cmd_z = float(z)
        self.last_twist = twist
        self.last_cmd_t = self.get_clock().now()

        # 주행 릴레이: 서버 발행률 그대로 즉시 중계(지연 최소화)
        if self.relay and twist is not None:
            out = TwistStamped()
            out.header.stamp = self.last_cmd_t.to_msg()
            out.header.frame_id = 'base_link'
            out.twist = twist
            self.drive_pub.publish(out)
            self.drive_zeroed = False

        if not self.armed and abs(self.cmd_z) > self.deadzone:
            # 첫 유효 명령 → 가정 위치(최하단)에서부터 램프 시작
            self.armed = True
            self.get_logger().info(
                f'첫 리프트 명령 수신 → 서보 활성화 (시작 위치 {self.cur_us:.0f}us 가정). '
                f'실제 위치가 다르면 그 차이만큼 한 번에 움직인다')

    # ---- 제어 루프 -------------------------------------------------------
    def _tick(self):
        z = self.cmd_z
        # 명령이 끊기면 정지(위치는 유지 = 짐 안 떨어짐)
        if self.last_cmd_t is None:
            z = 0.0
        else:
            age = (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                z = 0.0
                # 데드맨: 명령이 끊기면 주행 0속도를 한 번 쏜다(manual_cmd_gate 와 동일 동작).
                # 리프트는 여기서 정지만 하고 높이는 유지한다(짐 낙하 방지).
                if self.relay and not self.drive_zeroed:
                    stop = TwistStamped()
                    stop.header.stamp = self.get_clock().now().to_msg()
                    stop.header.frame_id = 'base_link'
                    self.drive_pub.publish(stop)
                    self.drive_zeroed = True
                    self.get_logger().warn(
                        f'명령 {age:.2f}s 끊김 → 주행 정지, 리프트 정지(높이 유지)')

        if not self.armed:
            self.backend.write(None)        # 아직 펄스 미발행(서보 무력)
            return

        now = self.get_clock().now()
        if self._last_move_t is None:
            self._last_move_t = now

        if abs(z) > self.deadzone:
            sign = self.dir_up * (1.0 if z > 0 else -1.0)
            step = self.ramp / self.rate * sign
            new_us = min(self.us_hi, max(self.us_lo, self.cur_us + step))

            if new_us == self.cur_us:
                # 소프트 리밋에 닿았는데도 계속 밀라는 명령이 온다.
                # ★2026-07-20 사고: 하드스톱에 물린 채 방치 → 서보 과열.
                #   더 버티지 말고 살짝 물러나 스톨을 푼다.
                if self._pin_since is None:
                    self._pin_since = now
                elif not self._backed_off and \
                        (now - self._pin_since).nanoseconds * 1e-9 > self.stall_timeout:
                    self.cur_us = min(self.us_hi, max(
                        self.us_lo, self.cur_us - sign * self.stall_backoff))
                    self._backed_off = True
                    self.get_logger().warn(
                        f'리프트 한계 도달({self.cur_us + sign * self.stall_backoff:.0f}us)에서 '
                        f'계속 밀림 → {self.stall_backoff:.0f}us 물러남(스톨 방지). '
                        f'us_down/us_up 재확인 필요')
            else:
                self._pin_since = None
                self._backed_off = False
                self.cur_us = new_us
                self._last_move_t = now
                self._released = False
        else:
            self._pin_since = None
            self._backed_off = False

        # 최하단 유휴 해제: 맨 아래에 오래 서 있으면 펄스를 끊어 발열을 없앤다.
        # 최하단에서는 짐이 기구에 얹혀 있으므로 무력화해도 떨어지지 않는다.
        # (들어올린 상태에서는 절대 해제하지 않는다 — 낙하한다.)
        if self.idle_release > 0 and \
                abs(self.cur_us - self.us_down) <= self.bottom_tol and \
                (now - self._last_move_t).nanoseconds * 1e-9 > self.idle_release:
            if not self._released:
                self._released = True
                self.get_logger().info(
                    f'최하단 {self.idle_release:.0f}s 유휴 → 펄스 해제(발열 방지). '
                    f'다음 명령에 자동 복귀')
            self.backend.write(None)
            self.pos_pub.publish(Float32(data=0.0))
            return

        self.backend.write(round(self.cur_us))

        frac = (self.cur_us - self.us_down) / (self.us_up - self.us_down) \
            if self.us_up != self.us_down else 0.0
        self.pos_pub.publish(Float32(data=float(max(0.0, min(1.0, frac)))))

    def destroy_node(self):
        self.backend.close()
        return super().destroy_node()


def main():
    rclpy.init()
    try:
        node = LiftServo()
    except RuntimeError as e:
        print(f'[pi_lift_servo] 기동 실패: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                # SIGINT/SIGTERM(=CLEAN_START pkill) 정상종료
    finally:
        node.destroy_node()                 # 여기서 펄스 정지 → 서보 무력화
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
