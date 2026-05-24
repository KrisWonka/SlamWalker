#!/usr/bin/env python3
"""
Serial bridge between ROS 2 Nav2 cmd_vel and Arduino motor controller.

Subscribes: /cmd_vel       (geometry_msgs/Twist)
            /motor_scales  (geometry_msgs/Vector3)  x=left, y=right
Publishes:  /odom          (nav_msgs/Odometry) + TF odom -> base_link

Arduino protocol:
  TX -> Arduino: "V<linear_x>,<angular_z>#\n"
  RX <- Arduino: "O<dTickL>,<dTickR>,<dt_ms>\n"
"""
import math
import os
import select
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wheel_base', 0.22)
        self.declare_parameter('ticks_per_meter', 0.0)
        self.declare_parameter('wheel_radius', 0.03575)
        self.declare_parameter('gear_ratio', 70.0)
        self.declare_parameter('encoder_cpr_on_a', 32.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Negate angular to fix turn direction if A/D are reversed
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('left_scale', 1.0)
        self.declare_parameter('right_scale', 1.0)
        self.declare_parameter('right_encoder_scale', 1.041)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value
        rate = self.get_parameter('rate_hz').value
        self.wheel_base = self.get_parameter('wheel_base').value
        tpm_direct = self.get_parameter('ticks_per_meter').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        gear_ratio = self.get_parameter('gear_ratio').value
        enc_cpr = self.get_parameter('encoder_cpr_on_a').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.invert_angular = self.get_parameter('invert_angular').value
        self.left_scale = self.get_parameter('left_scale').value
        self.right_scale = self.get_parameter('right_scale').value
        self.right_enc_scale = self.get_parameter('right_encoder_scale').value

        if tpm_direct > 0:
            self.ticks_per_meter = tpm_direct
            self.get_logger().info(
                f'ticks_per_meter = {tpm_direct:.1f} (direct / walking mechanism)')
        else:
            self.ticks_per_meter = (enc_cpr * gear_ratio) / (
                2.0 * math.pi * self.wheel_radius
            )
            self.get_logger().info(
                f'ticks_per_meter = {self.ticks_per_meter:.1f} (from radius={self.wheel_radius})')

        self.get_logger().info(
            f'invert_angular={self.invert_angular}  '
            f'scales L={self.left_scale:.2f} R={self.right_scale:.2f}')

        self.ser = None
        self._ser_lock = threading.Lock()
        self._open_serial()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        self.latest_twist = Twist()
        self.twist_lock = threading.Lock()
        self._last_twist_time = 0.0
        self._cmd_vel_timeout_s = 0.2

        self.create_subscription(Twist, 'cmd_vel', self._twist_cb, 10)
        self.create_subscription(Vector3, 'motor_scales', self._scales_cb, 10)
        self.create_timer(1.0 / rate, self._send_cmd)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True
        )
        self.reader_thread.start()

    def _open_serial(self):
        """Open (or reopen) the serial port with robust settings."""
        with self._ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            try:
                self.ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=0,
                    write_timeout=1,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
                time.sleep(2.0)
                self.ser.reset_input_buffer()
                self.get_logger().info(
                    f'Serial opened: {self._port} @ {self._baud}')
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Cannot open serial port {self._port}: {e}')
                self.ser = None

    def _twist_cb(self, msg: Twist):
        with self.twist_lock:
            self.latest_twist = msg
            self._last_twist_time = time.monotonic()

    def _scales_cb(self, msg: Vector3):
        if msg.x > 0:
            self.left_scale = msg.x
        if msg.y > 0:
            self.right_scale = msg.y
        self.get_logger().info(
            f'Motor scales updated: L={self.left_scale:.2f} R={self.right_scale:.2f}',
            throttle_duration_sec=0.5)

    def _send_cmd(self):
        with self._ser_lock:
            if self.ser is None:
                return
            with self.twist_lock:
                fresh = (time.monotonic() - self._last_twist_time) < self._cmd_vel_timeout_s
                if fresh:
                    lx = self.latest_twist.linear.x
                    az = self.latest_twist.angular.z
                else:
                    lx = 0.0
                    az = 0.0

            if self.invert_angular:
                az = -az

            # Apply per-wheel scaling if not both 1.0
            if self.left_scale != 1.0 or self.right_scale != 1.0:
                wb = self.wheel_base
                vL = lx - az * wb / 2.0
                vR = lx + az * wb / 2.0
                vL *= self.left_scale
                vR *= self.right_scale
                lx = (vL + vR) / 2.0
                az = (vR - vL) / wb

            frame = f'V{lx:.4f},{az:.4f}#\n'
            try:
                self.ser.write(frame.encode())
            except serial.SerialException:
                self.get_logger().warn('Serial write failed, will reconnect')
                self._schedule_reconnect()

    def _schedule_reconnect(self):
        """Schedule a reconnection attempt (call from any thread)."""
        def _reconnect():
            time.sleep(2.0)
            self.get_logger().info('Attempting serial reconnect...')
            self._open_serial()
        threading.Thread(target=_reconnect, daemon=True).start()

    def _serial_reader(self):
        """Read serial data using low-level os.read to tolerate CDC ACM
        glitches on Jetson that cause pyserial to falsely report disconnect."""
        buf = b''
        while rclpy.ok():
            with self._ser_lock:
                ser = self.ser
            if ser is None:
                time.sleep(1.0)
                continue

            try:
                fd = ser.fileno()
            except Exception:
                time.sleep(1.0)
                continue

            try:
                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue

                chunk = os.read(fd, 512)
                if not chunk:
                    # Empty read — CDC ACM glitch, just skip
                    time.sleep(0.01)
                    continue

                buf += chunk
                while b'\n' in buf:
                    line_bytes, buf = buf.split(b'\n', 1)
                    line = line_bytes.decode(errors='replace').strip()
                    if not line:
                        continue
                    self._dispatch_line(line)

            except OSError as e:
                if e.errno == 11:  # EAGAIN — non-blocking, no data yet
                    time.sleep(0.01)
                    continue
                self.get_logger().warn(f'Serial read OS error: {e}')
                time.sleep(0.5)
                buf = b''
            except Exception as e:
                self.get_logger().error(f'Reader exception: {e}')
                time.sleep(0.5)
                buf = b''

    def _dispatch_line(self, line: str):
        if line.startswith('O'):
            self._process_odom(line)
        elif line.startswith('D'):
            self.get_logger().info(f'Arduino: {line}', throttle_duration_sec=0.9)
        elif line == 'READY':
            self.get_logger().info('Arduino READY')
        elif line == 'OK':
            pass  # command acknowledged
        elif line.startswith('E:'):
            self.get_logger().warn(f'Arduino error: {line}', throttle_duration_sec=5.0)

    def _process_odom(self, line: str):
        try:
            parts = line[1:].split(',')
            d_ticks_l = int(parts[0])
            d_ticks_r = int(parts[1])
            dt_ms = int(parts[2])
        except (ValueError, IndexError):
            return

        dt = dt_ms / 1000.0
        if dt <= 0:
            return

        d_left = d_ticks_l / self.ticks_per_meter
        d_right = (d_ticks_r * self.right_enc_scale) / self.ticks_per_meter

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        if abs(d_ticks_l - d_ticks_r) <= 1:
            d_theta = 0.0

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.vx = d_center / dt
        self.vth = d_theta / dt

        now = self.get_clock().now().to_msg()
        q = self._yaw_to_quat(self.theta)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.pose.covariance[0] = 0.05   # x
        odom.pose.covariance[7] = 0.05   # y
        odom.pose.covariance[35] = 0.10  # yaw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance[0] = 0.05   # vx
        odom.twist.covariance[35] = 0.10  # vyaw
        self.odom_pub.publish(odom)

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node._ser_lock:
            if node.ser is not None:
                try:
                    node.ser.write(b'V0.0,0.0#\n')
                    node.ser.close()
                except Exception:
                    pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
