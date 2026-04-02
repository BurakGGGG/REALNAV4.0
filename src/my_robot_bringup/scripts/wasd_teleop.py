#!/usr/bin/env python3
"""
Direct Serial WASD Teleop + Odometry for STM32
================================================
- Doğrudan /dev/ttyAMA0 üzerinden STM32'ye L/R PWM komutları gönderir
- Encoder verisini okur, odometry hesaplar
- odom → base_footprint TF yayınlar
- /odom topic yayınlar
- ros2_control'ü BYPASS eder

STM32 Protokolü:
  Gönderilen: "L <pwm>\n"  ve  "R <pwm>\n"   (pwm: -255..+255)
  Alınan:     "ENC <dL> <dR> <dt_ms>\n"
"""

import sys
import os
import re
import time
import math
import termios
import tty
import select
import threading
import argparse

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

BANNER = """
╔════════════════════════════════╗
║   WASD  TELEOP + ODOM         ║
╠════════════════════════════════╣
║   W : İleri     S : Geri      ║
║   A : Sol       D : Sağ       ║
║   SPACE : Dur                 ║
║   + : Hız artır  - : Hız azalt║
║   Q : Çıkış                   ║
╚════════════════════════════════╝
"""

# Robot geometry
WHEEL_RADIUS = 0.051       # 320mm çevre → r=50.9mm
WHEEL_SEPARATION = 0.43    # Ölçüldü
TICKS_PER_REV = 4000
M_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

ENC_RE = re.compile(r"^ENC\s+(-?\d+)\s+(-?\d+)\s+(\d+)\s*$")


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TeleopOdom:
    def __init__(self, port='/dev/ttyAMA0', baud=115200):
        # Serial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"  Serial: {port} @ {baud}")

        # ROS
        rclpy.init()
        self.node = rclpy.create_node('teleop_odom')
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Motor state
        self.pwm = 60
        self.pwm_step = 15

        # Serial read thread
        self.buf = ""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()

        # Terminal
        self.old_settings = termios.tcgetattr(sys.stdin)

    def send_motor(self, l, r):
        try:
            self.ser.write(f"L {l}\n".encode('ascii'))
            time.sleep(0.002)
            self.ser.write(f"R {r}\n".encode('ascii'))
        except Exception as e:
            sys.stdout.write(f"\r  SERIAL ERR: {e}          ")
            sys.stdout.flush()

    def _read_serial_loop(self):
        """Background thread: read encoder data and publish odom."""
        while self.running:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    chunk = self.ser.read(n).decode('utf-8', errors='ignore')
                    self.buf += chunk

                    while "\n" in self.buf:
                        line, self.buf = self.buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        m = ENC_RE.match(line)
                        if m:
                            dL = int(m.group(1))
                            dR = int(m.group(2))
                            dt_ms = int(m.group(3))
                            self._handle_enc(dL, dR, dt_ms)
            except Exception:
                pass
            time.sleep(0.005)

    def _handle_enc(self, dL, dR, dt_ms):
        if dt_ms <= 0:
            return
        dt = dt_ms / 1000.0

        dist_l = dL * M_PER_TICK
        dist_r = dR * M_PER_TICK

        ds = 0.5 * (dist_r + dist_l)
        dth = (dist_r - dist_l) / WHEEL_SEPARATION

        th_mid = self.th + 0.5 * dth
        self.x += ds * math.cos(th_mid)
        self.y += ds * math.sin(th_mid)
        self.th += dth
        
        self.left_wheel_pos += (dL * M_PER_TICK) / WHEEL_RADIUS
        self.right_wheel_pos += (dR * M_PER_TICK) / WHEEL_RADIUS
        # Normalize
        while self.th > math.pi:
            self.th -= 2 * math.pi
        while self.th < -math.pi:
            self.th += 2 * math.pi

        self.last_v = ds / dt
        self.last_w = dth / dt

        self._publish_odom()

    def _publish_odom(self):
        now = self.node.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.th)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.angular.z = self.last_w
        self.odom_pub.publish(odom)

        # Joint states yayınla
        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]  # ayrı takip
        js.velocity = [self.last_v / WHEEL_RADIUS, self.last_v / WHEEL_RADIUS]
        js.effort = []
        self.joint_pub.publish(js)

    def get_key(self, timeout=0.1):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def run(self):
        print(BANNER)
        print(f'  PWM: {self.pwm}  (min=30, max=255)')
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                key = self.get_key(0.1)

                if key == 'w':
                    self.send_motor(self.pwm, int(self.pwm * 1.046))
                elif key == 's':
                    self.send_motor(-self.pwm, int(-self.pwm * 1.046))
                elif key == 'a':
                    self.send_motor(-self.pwm, int(self.pwm * 1.046))
                elif key == 'd':
                    self.send_motor(self.pwm, int(-self.pwm * 1.046))
                elif key == ' ':
                    self.send_motor(0, 0)
                elif key in ('+', '='):
                    self.pwm = min(self.pwm + self.pwm_step, 255)
                    sys.stdout.write(f'\r  PWM: {self.pwm}          ')
                    sys.stdout.flush()
                    continue
                elif key == '-':
                    self.pwm = max(self.pwm - self.pwm_step, 30)
                    sys.stdout.write(f'\r  PWM: {self.pwm}          ')
                    sys.stdout.flush()
                    continue
                elif key == 'q' or key == '\x03':
                    break
                elif key == '':
                    self.send_motor(0, 0)

                rclpy.spin_once(self.node, timeout_sec=0)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.send_motor(0, 0)
            time.sleep(0.1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.ser.close()
            self.node.destroy_node()
            rclpy.shutdown()
            print('\n  Robot durdu. Çıkış.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    teleop = TeleopOdom(port=args.port, baud=args.baud)
    teleop.run()


if __name__ == '__main__':
    main()
