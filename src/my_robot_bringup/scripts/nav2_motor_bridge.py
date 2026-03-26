#!/usr/bin/env python3
"""
Nav2 Motor Bridge için gerçek robot (STM32) haberleşme düğümü.
================================================================
- `/cmd_vel` alır -> Kinematik -> PWM (L/R) -> Serial Port
- Serial Port'tan "ENC dL dR dt_ms" okur -> Odometry -> `/odom` TF & Topic
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

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# Robot geometry
WHEEL_RADIUS = 0.050       # m  (STM32: WHEEL_DIAM_MM=100 → r=50mm)
WHEEL_SEPARATION = 0.43    # m  (URDF base_width ile eşleşmeli — URDF doğru!)
TICKS_PER_REV = 1600       # Kalibre: 572→2.8m, 572×2.805≈1600 (400 PPR × 4x quadrature)
M_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

ENC_RE = re.compile(r"^ENC\s+(-?\d+)\s+(-?\d+)\s+(\d+)\s*$")

# PWM Tuning
# Robotun gerçek hayatta 1.0 m/s hıza ulaşması için gereken PWM değeri
MAX_LINEAR_VEL = 0.26  # m/s — Nav2 DWB max_vel_x ile eşleşmeli
MAX_PWM = 255.0
# Kalibre: Minicom'da PWM=100 → 1.09 m/s → PWM/hız = 92
# Nav2 0.26 m/s gönderince: 0.26 × 92 = PWM 24 → gerçek hız ~0.26 m/s ✓

def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

class Nav2MotorBridge(Node):
    def __init__(self):
        super().__init__('nav2_motor_bridge')
        
        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("pwm_multiplier", int(MAX_PWM / MAX_LINEAR_VEL)) 
        
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.pwm_multiplier = self.get_parameter("pwm_multiplier").value

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)  # 200ms timeout — readline() satır sonunu beklesin!
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

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        
        # Joint states and wheels positions
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        # Serial read thread
        self.buf = ""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()
        
        # Odom and TF publishing timer (30 Hz CPU Optimize)
        self.create_timer(1.0 / 30.0, self._publish_timer_callback)
        
        # Odom watchdog (Eğer serial'den veri gelmezse 0 hızı yayını yapmak için)
        self.last_enc_time = time.time()
        self.create_timer(0.5, self._watchdog_timer)
        
        self.get_logger().info("Nav2 Motor Bridge HAZIR!")
        self.get_logger().info(f"Hız Çarpanı (PWM Multiplier): {self.pwm_multiplier}")

    def cmd_vel_callback(self, msg: Twist):
        """Nav2'den gelen cmd_vel mesajını al, PWM'e çevir, serial'dan yolla."""
        v = msg.linear.x    # m/s
        w = msg.angular.z   # rad/s
        
        # Diferansiyel Sürüş Kinematiği
        # v_L = v - (w * L / 2)
        # v_R = v + (w * L / 2)
        v_l = v - (w * WHEEL_SEPARATION / 2.0)
        v_r = v + (w * WHEEL_SEPARATION / 2.0)
        
        # Metre/Saniye hızını PWM sinyaline oranla
        pwm_l = int(v_l * self.pwm_multiplier)
        pwm_r = int(v_r * self.pwm_multiplier)
        
        # Limit the PWM (max 255, min -255)
        # 30'un altındaki (ölü bölge) PWM'leri robot hareket etmeyeceği için 0 kabul ediyoruz (eğer komut 0'a yakınsa).
        if abs(pwm_l) > 255: pwm_l = int(math.copysign(255, pwm_l))
        if abs(pwm_r) > 255: pwm_r = int(math.copysign(255, pwm_r))
        
        if abs(pwm_l) < 5 and abs(pwm_r) < 5:
            pwm_l, pwm_r = 0, 0
            
        self._send_motor(pwm_l, pwm_r)

    def _send_motor(self, l, r):
        try:
            self.ser.write(f"L {l}\n".encode('ascii'))
            time.sleep(0.002)
            self.ser.write(f"R {r}\n".encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial Write Hatası: {e}")

    def _read_serial_loop(self):
        """Arka planda UART'ı dinler, ENC verilerini çözer ve state'i günceller.
           Buffer tabanlı okuma — veri kaybı olmaz."""
        buf = b""
        while self.running and rclpy.ok():
            try:
                # Mevcut tüm byte'ları oku (non-blocking kontrol)
                n = self.ser.in_waiting
                if n > 0:
                    chunk = self.ser.read(n)
                    if chunk:
                        buf += chunk

                    # Buffer'da tam satırlar varsa işle
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
                    time.sleep(0.005)  # 5ms bekle — CPU koruma
            except serial.SerialException:
                time.sleep(0.1)
            except Exception:
                pass

    def _handle_enc(self, dL, dR, dt_ms):
        self.last_enc_time = time.time()
        
        if dt_ms <= 0:
            return
        dt = dt_ms / 1000.0

        # Encoder drift deadband — dururken gürültüyü filtrele
        if abs(dL) < 2 and abs(dR) < 2:
            dL = 0
            dR = 0

        dist_l = dL * M_PER_TICK
        dist_r = dR * M_PER_TICK

        ds = 0.5 * (dist_r + dist_l)
        dth = (dist_r - dist_l) / WHEEL_SEPARATION

        th_mid = self.th + 0.5 * dth
        self.x += ds * math.cos(th_mid)
        self.y += ds * math.sin(th_mid)
        self.th += dth
        
        # Normalize
        while self.th > math.pi: self.th -= 2 * math.pi
        while self.th < -math.pi: self.th += 2 * math.pi

        self.last_v = ds / dt
        self.last_w = dth / dt
        
        # Hız deadband — çok küçük hızları sıfırla
        if abs(self.last_v) < 0.01:
            self.last_v = 0.0
        if abs(self.last_w) < 0.01:
            self.last_w = 0.0
        
        # Tekerleklerin radyan cinsinden konumları (JointStates için)
        self.left_wheel_pos += (dL / TICKS_PER_REV) * 2 * math.pi
        self.right_wheel_pos += (dR / TICKS_PER_REV) * 2 * math.pi
        self.left_wheel_vel = (dL / TICKS_PER_REV * 2 * math.pi) / dt
        self.right_wheel_vel = (dR / TICKS_PER_REV * 2 * math.pi) / dt
        
        # Not: Yayını burada YAPMIYORUZ. _publish_timer_callback ilgileniyor.

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
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
        
        # Covariance — SLAM'ın odometriyi körü körüne takip etmemesi için!
        # Sıfır covariance = "odom %100 doğru" demek, SLAM düzeltme yapamaz.
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
        
        # Joint states message (Tekerlek TF'leri için şart)
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [self.left_wheel_vel, self.right_wheel_vel]
        self.joint_pub.publish(joint_state)

    def _publish_timer_callback(self):
        """Sabit frekansta (30Hz) odom ve TF yayını yapar."""
        self._publish_odom()
        
    def _watchdog_timer(self):
        """Eğer STM32'den 0.5 saniyedir veri gelmiyorsa hızları 0'la."""
        if time.time() - self.last_enc_time > 0.5:
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
            pass  # Zaten shutdown olmuş olabilir (SIGINT race condition)

if __name__ == '__main__':
    main()
