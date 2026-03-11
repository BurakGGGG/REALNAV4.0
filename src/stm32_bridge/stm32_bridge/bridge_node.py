#!/usr/bin/env python3
import math
import re
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# pyserial
import serial


def yaw_to_quat(yaw: float):
    """Return (x,y,z,w) quaternion for yaw around Z."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Stm32Bridge(Node):
    ENC_RE = re.compile(r"^ENC\s+(-?\d+)\s+(-?\d+)\s+(\d+)\s*$")

    def __init__(self):
        super().__init__('stm32_bridge')

        # -------- Parameters --------
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.055)       # meters (URDF ile eşleşmeli: 5.5cm)
        self.declare_parameter('wheel_separation', 0.55)    # meters (URDF ile eşleşmeli: tekerlekler arası 55cm)

        # Encoder scale
        # Burada EN KRİTİK parametre: 1 teker turu kaç count?
        # Encoder CPR * 4 (quadrature) * gear_ratio gibi düşün.
        self.declare_parameter('ticks_per_wheel_rev', 1000)  # ÖRNEK! seninki farklı olabilir

        # Direction fixes
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # Publish options
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # STM command protocol (bizim öneri)
        # STM tarafına: "CMD <v_mm_s> <w_mrad_s>\n"
        self.declare_parameter('send_cmd', True)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_ms', 300)

        # -------- Read parameters --------
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_wheel_rev = int(self.get_parameter('ticks_per_wheel_rev').value)

        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.send_cmd = bool(self.get_parameter('send_cmd').value)
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.cmd_timeout_ms = int(self.get_parameter('cmd_timeout_ms').value)

        # meters per tick
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / float(self.ticks_per_wheel_rev)

        self.get_logger().info(f"port={self.port} baud={self.baud}")
        self.get_logger().info(f"frames: odom_frame={self.odom_frame} base_frame={self.base_frame}")
        self.get_logger().info(f"wheel_radius={self.wheel_radius} wheel_separation={self.wheel_separation}")
        self.get_logger().info(f"ticks_per_wheel_rev={self.ticks_per_wheel_rev} => m_per_tick={self.m_per_tick:.9f}")
        self.get_logger().info(f"invert_left={self.invert_left} invert_right={self.invert_right}")

        # -------- Serial open --------
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)  # non-blocking
        except Exception as e:
            self.get_logger().fatal(f"Cannot open serial {self.port}: {e}")
            raise

        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        # -------- ROS pub/sub --------
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)

        # -------- State --------
        self.buf = b""

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_enc_time = time.time()
        self.enc_lines = 0

        self.last_cmd = Twist()
        self.last_cmd_stamp = self.get_clock().now()

        # Timers
        self.read_timer = self.create_timer(0.01, self.read_serial)  # 100 Hz read poll
        self.diag_timer = self.create_timer(1.0, self.print_diag)

        if self.send_cmd:
            self.cmd_timer = self.create_timer(1.0 / self.cmd_rate_hz, self.send_cmd_timer)

    # ---------- CMD VEL ----------
    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_stamp = self.get_clock().now()

    def send_cmd_timer(self):
        # timeout -> stop
        now = self.get_clock().now()
        age_ms = (now - self.last_cmd_stamp).nanoseconds / 1e6

        v = self.last_cmd.linear.x
        w = self.last_cmd.angular.z
        if age_ms > self.cmd_timeout_ms:
            v = 0.0
            w = 0.0

        # convert to STM protocol: mm/s and mrad/s
        v_mm = int(round(v * 1000.0))
        w_mrad = int(round(w * 1000.0))

        line = f"CMD {v_mm} {w_mrad}\n".encode('ascii', errors='ignore')
        try:
            self.ser.write(line)
        except Exception as e:
            self.get_logger().warn(f"serial write failed: {e}")

    # ---------- SERIAL READ + ODOM ----------
    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            chunk = self.ser.read(n)
            if not chunk:
                return
            self.buf += chunk
        except Exception as e:
            self.get_logger().warn(f"serial read failed: {e}")
            return

        # split lines
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.strip().decode('utf-8', errors='ignore')
            if not line:
                continue

            m = self.ENC_RE.match(line)
            if m:
                dL = int(m.group(1))
                dR = int(m.group(2))
                dt_ms = int(m.group(3))
                self.handle_enc(dL, dR, dt_ms)
            else:
                # İstersen debug satırlarını burada loglayabilirsin ama spam olmasın diye kapalı:
                # self.get_logger().debug(f"IGN: {line}")
                pass

    def handle_enc(self, dL: int, dR: int, dt_ms: int):
        self.enc_lines += 1
        self.last_enc_time = time.time()

        if self.invert_left:
            dL = -dL
        if self.invert_right:
            dR = -dR

        if dt_ms <= 0:
            return
        dt = dt_ms / 1000.0

        # ticks -> meters
        distL = dL * self.m_per_tick
        distR = dR * self.m_per_tick

        ds = 0.5 * (distR + distL)
        dth = (distR - distL) / self.wheel_separation

        # integrate (midpoint)
        th_mid = self.th + 0.5 * dth
        self.x += ds * math.cos(th_mid)
        self.y += ds * math.sin(th_mid)
        self.th = normalize_angle(self.th + dth)

        v = ds / dt
        w = dth / dt

        self.publish_odom(v, w)

    def publish_odom(self, v: float, w: float):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def print_diag(self):
        age = time.time() - self.last_enc_time
        self.get_logger().info(
            f"ENC lines={self.enc_lines} (last {age:.2f}s ago) pose=({self.x:.3f},{self.y:.3f},{self.th:.3f}rad)"
        )


def main():
    rclpy.init()
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.ser.close()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
