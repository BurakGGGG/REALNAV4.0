#!/usr/bin/env python3
"""
STM32 Serial Bridge Node
STM32 ile UART üzerinden haberleşir, ROS2 topic'leri ile entegre eder.

Görevler:
1. /cmd_vel topic'ini dinler → STM32'ye "C:linear,angular\n" gönderir
2. STM32'den "E:left,right\n" okur → /joint_states yayınlar
3. Odometry hesaplar → /odom yayınlar (opsiyonel, controller kullanıyorsa gerekmez)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import serial
import threading
import time
import math


class STM32SerialBridge(Node):
    """
    STM32 ile serial haberleşme yapan ROS2 node'u.
    """

    def __init__(self):
        super().__init__('stm32_serial_bridge')

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("wheel_separation", 0.55)
        self.declare_parameter("wheel_radius", 0.055)
        self.declare_parameter("encoder_cpr", 4000)

        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        timeout = self.get_parameter("timeout").value

        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.encoder_cpr = self.get_parameter("encoder_cpr").value

        # Serial port
        self.serial = None
        self.serial_lock = threading.Lock()

        # Encoder değerleri (thread-safe erişim için encoder_lock kullan)
        self.encoder_lock = threading.Lock()
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_encoder_time = time.time()

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = time.time()

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10
        )

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Timer for reading encoder data (20 Hz)
        self.create_timer(0.05, self.read_encoder_timer_callback)

        # Connect to serial port
        try:
            self.serial = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(
                f"Connected to {serial_port} at {baud_rate} baud"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to connect to {serial_port}: {e}"
            )
            self.get_logger().error(
                "Make sure STM32 is connected and serial port is correct"
            )
            # Node'u kapat, serial bağlantısı olmadan çalışamaz
            raise RuntimeError(f"Failed to connect to serial port: {serial_port}")

        # Start reading thread
        self.read_thread = threading.Thread(target=self.read_serial_thread)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.get_logger().info("STM32 Serial Bridge node started!")

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel topic'inden gelen komutları STM32'ye gönderir.
        Format: "C:linear,angular\n"
        """
        if self.serial is None or not self.serial.is_open:
            return

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Komutu formatla: "C:linear,angular\n"
        cmd = f"C:{linear_vel:.3f},{angular_vel:.3f}\n"

        try:
            with self.serial_lock:
                self.serial.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def read_serial_thread(self):
        """
        Serial port'tan sürekli okuma yapan thread.
        STM32'den gelen "E:left,right\n" mesajlarını parse eder.
        """
        while rclpy.ok() and self.serial is not None and self.serial.is_open:
            try:
                # Serial'den satır oku
                line = self.serial.readline().decode('ascii', errors='ignore').strip()

                if line.startswith('E:'):
                    # Encoder verisi: "E:left,right"
                    try:
                        parts = line[2:].split(',')
                        if len(parts) == 2:
                            left_enc = int(parts[0])
                            right_enc = int(parts[1])
                            
                            # Encoder değerlerini güncelle (thread-safe)
                            with self.encoder_lock:
                                self.left_encoder_count = left_enc
                                self.right_encoder_count = right_enc
                    except ValueError as e:
                        self.get_logger().warn(f"Failed to parse encoder data: {line}")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read thread: {e}")
                time.sleep(0.1)

    def read_encoder_timer_callback(self):
        """
        Timer callback: Encoder verilerini /joint_states ve /odom olarak yayınlar.
        20 Hz çağrılır.
        """
        if self.serial is None or not self.serial.is_open:
            return

        # Joint states yayınla
        self.publish_joint_states()

        # Odometry yayınla (opsiyonel, controller kullanıyorsa gerekmez)
        # self.publish_odometry()

    def publish_joint_states(self):
        """
        Encoder verilerinden /joint_states yayınlar.
        ROS2 Control için gerekli.
        """
        current_time = time.time()
        dt = current_time - self.last_encoder_time

        if dt <= 0:
            return

        # Encoder farklarını hesapla (thread-safe okuma)
        with self.encoder_lock:
            left_enc = self.left_encoder_count
            right_enc = self.right_encoder_count

        left_diff = left_enc - self.last_left_encoder
        right_diff = right_enc - self.last_right_encoder

        # Encoder'ı radyan'a çevir
        # position: toplam açı (radyan)
        # velocity: açısal hız (rad/s)
        left_position = (left_enc / self.encoder_cpr) * 2.0 * math.pi
        right_position = (right_enc / self.encoder_cpr) * 2.0 * math.pi

        left_velocity = (left_diff / self.encoder_cpr) * 2.0 * math.pi / dt
        right_velocity = (right_diff / self.encoder_cpr) * 2.0 * math.pi / dt

        # JointState mesajı oluştur
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ""
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_position, right_position]
        joint_state.velocity = [left_velocity, right_velocity]
        joint_state.effort = []

        self.joint_state_pub.publish(joint_state)

        # Güncelle
        self.last_left_encoder = left_enc
        self.last_right_encoder = right_enc
        self.last_encoder_time = current_time

    def publish_odometry(self):
        """
        Encoder verilerinden odometry hesaplar ve /odom yayınlar.
        NOT: Controller kullanıyorsanız bu fonksiyon gerekmez,
        controller zaten odometry hesaplıyor.
        """
        current_time = time.time()
        dt = current_time - self.last_odom_time

        if dt <= 0:
            return

        # Encoder farklarını hesapla
        left_diff = self.left_encoder_count - self.last_left_encoder
        right_diff = self.right_encoder_count - self.last_right_encoder

        # Encoder'ı mesafeye çevir (metre)
        left_distance = (left_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius
        right_distance = (right_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius

        # Differential drive odometry
        linear_vel = (right_distance + left_distance) / 2.0 / dt
        angular_vel = (right_distance - left_distance) / self.wheel_separation / dt

        # Pozisyon güncelle
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        self.theta += angular_vel * dt

        # Odometry mesajı oluştur
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Covariance (basit)
        odom.pose.covariance[0] = 0.001  # x
        odom.pose.covariance[7] = 0.001  # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.001  # vx
        odom.twist.covariance[35] = 0.01  # vyaw

        self.odom_pub.publish(odom)
        self.last_odom_time = current_time

    def destroy_node(self):
        """Node kapanırken serial port'u kapat."""
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main():
    rclpy.init()
    node = STM32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
