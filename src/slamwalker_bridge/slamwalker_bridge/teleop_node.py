#!/usr/bin/env python3
"""
SlamWalker keyboard teleop.

  W       : forward
  S       : backward
  A       : in-place left turn  (left reverse, right forward)
  D       : in-place right turn (left forward, right reverse)
  Space   : stop
  ↑/↓     : increase / decrease speed
  ←/→     : increase / decrease turn rate
  [/]     : left motor scale  -/+
  ,/.     : right motor scale -/+
  Q/ESC   : quit
"""
import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'

SPEED_STEP = 0.10
TURN_STEP = 0.5
SCALE_STEP = 0.05


class TeleopNode(Node):
    def __init__(self):
        super().__init__('slamwalker_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scale_pub = self.create_publisher(Vector3, 'motor_scales', 10)
        self.timer = self.create_timer(0.1, self._publish)

        self.max_lin = 0.60
        # 0.22m wheel_base with az=6.0 gives near full opposite wheel speeds.
        self.max_ang = 6.0
        self.lin_val = 0.0   # -1, 0, 1
        self.ang_val = 0.0   # -1, 0, 1
        self.left_scale = 1.0
        self.right_scale = 1.0

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.lin_val * self.max_lin
        msg.angular.z = self.ang_val * self.max_ang
        self.pub.publish(msg)

    def _publish_scales(self):
        msg = Vector3()
        msg.x = self.left_scale
        msg.y = self.right_scale
        self.scale_pub.publish(msg)

    def adjust_speed(self, delta):
        self.max_lin = max(0.05, min(2.00, self.max_lin + delta))

    def adjust_turn(self, delta):
        self.max_ang = max(0.5, min(8.0, self.max_ang + delta))

    def adjust_left_scale(self, delta):
        self.left_scale = max(0.10, min(2.00, self.left_scale + delta))
        self._publish_scales()

    def adjust_right_scale(self, delta):
        self.right_scale = max(0.10, min(2.00, self.right_scale + delta))
        self._publish_scales()

    def stop(self):
        self.lin_val = 0.0
        self.ang_val = 0.0


def get_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        ch2 = sys.stdin.read(1)
        ch3 = sys.stdin.read(1)
        return ch + ch2 + ch3
    return ch


def print_banner():
    print("\r\033[2J\033[H", end='')
    print("╔════════════════════════════════════════════════╗")
    print("║              SlamWalker Teleop                 ║")
    print("╠════════════════════════════════════════════════╣")
    print("║  W         : forward                          ║")
    print("║  S         : backward                         ║")
    print("║  A         : in-place left turn               ║")
    print("║  D         : in-place right turn              ║")
    print("║  Space     : stop                              ║")
    print("║  Q / ESC   : quit                              ║")
    print("║  ↑ / ↓     : speed  +/- 0.10                  ║")
    print("║  ← / →     : turn   +/- 0.5                   ║")
    print("║  [ / ]     : LEFT  motor scale +/- 0.05       ║")
    print("║  , / .     : RIGHT motor scale +/- 0.05       ║")
    print("╚════════════════════════════════════════════════╝")
    print()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        print_banner()
        print(f'\r  Speed: {node.max_lin:.2f}  Turn: {node.max_ang:.1f}'
              f'  | L: {node.left_scale:.2f}  R: {node.right_scale:.2f}'
              f'  | STOPPED\r', end='', flush=True)

        while True:
            key = get_key()

            if key.lower() == 'q':
                node.stop()
                break
            elif key == '\x1b' and len(key) == 1:
                node.stop()
                break
            elif key == ' ':
                node.stop()
            elif key == 'w':
                node.lin_val = 1.0
                node.ang_val = 0.0
            elif key == 's':
                node.lin_val = -1.0
                node.ang_val = 0.0
            elif key == 'a':
                node.lin_val = 0.0
                node.ang_val = 1.0
            elif key == 'd':
                node.lin_val = 0.0
                node.ang_val = -1.0
            elif key == ARROW_UP:
                node.adjust_speed(SPEED_STEP)
            elif key == ARROW_DOWN:
                node.adjust_speed(-SPEED_STEP)
            elif key == ARROW_LEFT:
                node.adjust_turn(TURN_STEP)
            elif key == ARROW_RIGHT:
                node.adjust_turn(-TURN_STEP)
            elif key == '[':
                node.adjust_left_scale(-SCALE_STEP)
            elif key == ']':
                node.adjust_left_scale(SCALE_STEP)
            elif key == ',':
                node.adjust_right_scale(-SCALE_STEP)
            elif key == '.':
                node.adjust_right_scale(SCALE_STEP)
            else:
                continue

            vx = node.lin_val * node.max_lin
            wz = node.ang_val * node.max_ang
            if node.lin_val != 0 or node.ang_val != 0:
                motion = f'vx={vx:+.2f} wz={wz:+.1f}'
            else:
                motion = 'STOPPED'
            status = (
                f'  Speed: {node.max_lin:.2f}  Turn: {node.max_ang:.1f}'
                f'  | L: {node.left_scale:.2f}  R: {node.right_scale:.2f}'
                f'  | {motion}')
            print(f'\r\033[K{status}\r', end='', flush=True)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print('\nBye!')


if __name__ == '__main__':
    main()
