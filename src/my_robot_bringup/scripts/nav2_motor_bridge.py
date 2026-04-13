#!/usr/bin/env python3
"""
Nav2 Motor Bridge — STM32 haberleşme düğümü (DÜZELTİLMİŞ)
================================================================
- `/cmd_vel` alır -> Kinematik -> PWM (L/R) -> Serial Port
- Serial Port'tan "ENC dL dR dt_ms" okur -> Odometry -> `/odom` TF & Topic

DÜZELTİLMİŞ SORUNLAR:
- Encoder çarpanı: sağ encoder kalibrasyonu (1.044x)
- Goal reached sonrası spin: agresif deadzone + zamanlayıcı tabanlı durma
- Watchdog: 0.2s timeout, 0.1s kontrol
"""

import sys
import os
import re
import time
import math
import threading

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Defaults
DEFAULT_WHEEL_RADIUS = 0.051
DEFAULT_WHEEL_SEPARATION = 0.43
DEFAULT_TICKS_PER_REV = 4000

# Sağ encoder kalibrasyon çarpanı
RIGHT_ENCODER_MULTIPLIER = 1.044

ENC_RE = re.compile(r"^ENC\s+(-?\d+)\s+(-?\d+)\s+(\d+)\s*$")

MAX_PWM = 255.0

def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

class Nav2MotorBridge(Node):
    def __init__(self):
        super().__init__('nav2_motor_bridge')
        
        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("pwm_multiplier", 200)
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("wheel_separation", DEFAULT_WHEEL_SEPARATION)
        self.declare_parameter("ticks_per_rev", DEFAULT_TICKS_PER_REV)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.pwm_multiplier = self.get_parameter("pwm_multiplier").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Serial BAĞLANDI: {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial BAĞLANTI HATASI: {e}")
            sys.exit(1)

        # Publishers & Broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber — /cmd_vel dinle
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.velocity_alpha = 0.3
        
        # Joint states
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        # Thread safety
        self.odom_lock = threading.Lock()

        # Serial read thread
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()
        
        # Odom publishing timer (30 Hz)
        self.create_timer(1.0 / 30.0, self._publish_timer_callback)
        
        # ===== DÜZELTME: Motor durma kontrolü =====
        # Son gönderilen PWM değerlerini takip et
        self._current_pwm_l = 0
        self._current_pwm_r = 0
        # Son sıfır olmayan komut zamanı
        self._last_nonzero_cmd_time = time.time()
        # Consecutive zero command counter
        self._zero_cmd_count = 0
        
        # Watchdog timer (100ms)
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self._watchdog_timer)
        
        self.get_logger().info("Nav2 Motor Bridge HAZIR!")
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius}, "
            f"wheel_separation={self.wheel_separation}, "
            f"ticks_per_rev={self.ticks_per_rev}, "
            f"pwm_multiplier={self.pwm_multiplier}, "
            f"right_encoder_multiplier={RIGHT_ENCODER_MULTIPLIER}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        v = msg.linear.x
        w = msg.angular.z
        
        # ===== DÜZELTME: Agresif deadzone =====
        # Çok küçük komutlar = Nav2 durma/dönme aşamasında
        # Bunları tamamen yok say ve motoru durdur
        if abs(v) < 0.03 and abs(w) < 0.25:
            self._zero_cmd_count += 1
            # 3 ardışık sıfır-benzeri komut = gerçekten dur
            if self._zero_cmd_count >= 3:
                if self._current_pwm_l != 0 or self._current_pwm_r != 0:
                    self._send_motor(0, 0)
                    self._current_pwm_l = 0
                    self._current_pwm_r = 0
            return
        
        # Sıfır olmayan komut geldi, counter resetle
        self._zero_cmd_count = 0
        self._last_nonzero_cmd_time = time.time()
        
        v_l = v - (w * self.wheel_separation / 2.0)
        v_r = v + (w * self.wheel_separation / 2.0)
        
        pwm_l = int(v_l * self.pwm_multiplier)
        pwm_r = int(v_r * self.pwm_multiplier * 1.046)
        
        if abs(pwm_l) > 255: pwm_l = int(math.copysign(255, pwm_l))
        if abs(pwm_r) > 255: pwm_r = int(math.copysign(255, pwm_r))
        
        # Minimum PWM eşiği
        MIN_PWM = 35
        if pwm_l != 0 and abs(pwm_l) < MIN_PWM:
            pwm_l = int(math.copysign(MIN_PWM, pwm_l))
        if pwm_r != 0 and abs(pwm_r) < MIN_PWM:
            pwm_r = int(math.copysign(MIN_PWM, pwm_r))
        
        self._send_motor(pwm_l, pwm_r)
        self._current_pwm_l = pwm_l
        self._current_pwm_r = pwm_r

    def _send_motor(self, l, r):
        try:
            self.ser.write(f"L {l}\n".encode('ascii'))
            time.sleep(0.002)
            self.ser.write(f"R {r}\n".encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial Write Hatası: {e}")

    def _read_serial_loop(self):
        buf = b""
        while self.running and rclpy.ok():
            try:
                n = self.ser.in_waiting
                if n > 0:
                    chunk = self.ser.read(n)
                    if chunk:
                        buf += chunk
                    while b"\n" in buf:
                        line_bytes, buf = buf.split(b"\n", 1)
                        line = line_bytes.strip().decode('utf-8', errors='ignore')
                        if not line:
                            continue
                        m = ENC_RE.match(line)
                        if m:
                            dL = int(m.group(1))
                            dR = int(m.group(2))
                            dt_ms = int(m.group(3))
                            self._handle_enc(dL, dR, dt_ms)
                else:
                    time.sleep(0.005)
            except serial.SerialException:
                time.sleep(0.1)
            except Exception:
                pass

    def _handle_enc(self, dL, dR, dt_ms):
        if dt_ms <= 0:
            return
        dt = dt_ms / 1000.0

        # Encoder drift deadband
        if abs(dL) < 2 and abs(dR) < 2:
            dL = 0
            dR = 0

        dist_l = dL * self.m_per_tick
        # ===== DÜZELTME: Sağ encoder kalibrasyon çarpanı =====
        dist_r = dR * self.m_per_tick * RIGHT_ENCODER_MULTIPLIER

        ds = 0.5 * (dist_r + dist_l)
        dth = (dist_r - dist_l) / self.wheel_separation

        with self.odom_lock:
            th_mid = self.th + 0.5 * dth
            self.x += ds * math.cos(th_mid)
            self.y += ds * math.sin(th_mid)
            self.th += dth

            while self.th > math.pi: self.th -= 2 * math.pi
            while self.th < -math.pi: self.th += 2 * math.pi

            raw_v = ds / dt
            raw_w = dth / dt

            self.last_v = self.velocity_alpha * raw_v + (1.0 - self.velocity_alpha) * self.last_v
            self.last_w = self.velocity_alpha * raw_w + (1.0 - self.velocity_alpha) * self.last_w

            if abs(self.last_v) < 0.01:
                self.last_v = 0.0
            if abs(self.last_w) < 0.01:
                self.last_w = 0.0

            # Joint positions — sağ encoder çarpanı uygulanmış mesafe
            self.left_wheel_pos += (dL / self.ticks_per_rev) * 2 * math.pi
            self.right_wheel_pos += (dR * RIGHT_ENCODER_MULTIPLIER / self.ticks_per_rev) * 2 * math.pi
            self.left_wheel_vel = (dL / self.ticks_per_rev * 2 * math.pi) / dt
            self.right_wheel_vel = (dR * RIGHT_ENCODER_MULTIPLIER / self.ticks_per_rev * 2 * math.pi) / dt

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
        with self.odom_lock:
            x, y, th = self.x, self.y, self.th
            v, w = self.last_v, self.last_w
            lp, rp = self.left_wheel_pos, self.right_wheel_pos
            lv, rv = self.left_wheel_vel, self.right_wheel_vel
        qx, qy, qz, qw = yaw_to_quat(th)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
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
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.1, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.1, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.1, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.2,
        ]
        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.1, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.1, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.1, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.2,
        ]
        self.odom_pub.publish(odom)
        
        # Joint states
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [lp, rp]
        joint_state.velocity = [lv, rv]
        self.joint_pub.publish(joint_state)

    def _publish_timer_callback(self):
        self._publish_odom()
        
    def _watchdog_timer(self):
        """0.2 saniye cmd_vel gelmezse motoru durdur"""
        now = time.time()
        
        # Son komuttan bu yana geçen süre
        elapsed = now - self.last_cmd_time
        
        if elapsed > 0.2:
            if self._current_pwm_l != 0 or self._current_pwm_r != 0:
                self._send_motor(0, 0)
                self._current_pwm_l = 0
                self._current_pwm_r = 0
            with self.odom_lock:
                self.last_v = 0.0
                self.last_w = 0.0
                self.left_wheel_vel = 0.0
                self.right_wheel_vel = 0.0

    def destroy_node(self):
        self.running = False
        self._send_motor(0, 0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = Nav2MotorBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
