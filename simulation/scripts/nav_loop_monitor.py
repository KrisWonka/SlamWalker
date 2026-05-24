#!/usr/bin/env python3
import math
import sys
import termios
import tty
import select
import threading
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray


@dataclass
class SharedState:
    sim_ok: bool = False
    loc_ok: bool = False
    nav_ok: bool = False
    amcl_x: float | None = None
    amcl_y: float | None = None
    cmd_vx: float | None = None
    cmd_wz: float | None = None
    odom_vx: float | None = None
    odom_wz: float | None = None
    goal_x: float | None = None
    goal_y: float | None = None
    nav_status_code: int | None = None
    nav_status_text: str = "NO_GOAL"
    last_goal_time: datetime | None = None
    last_status_time: datetime | None = None
    last_odom_time: datetime | None = None


def status_from_code(code: int | None) -> str:
    mapping = {
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    if code is None:
        return "NO_GOAL"
    return mapping.get(code, f"UNKNOWN({code})")


def fmt(v: float | None, ndigits: int = 3) -> str:
    if v is None:
        return "n/a"
    return f"{v:.{ndigits}f}"


class NavLoopMonitor(Node):
    def __init__(self, world_name: str, map_path: str):
        super().__init__("nav_loop_monitor")
        self.world_name = world_name
        self.map_path = map_path
        self.state = SharedState()
        self._lock = threading.Lock()
        self._shutdown = False

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, amcl_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, default_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, default_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, default_qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_nav_status, default_qos)

    def request_stop(self):
        with self._lock:
            self._shutdown = True

    def should_stop(self) -> bool:
        with self._lock:
            return self._shutdown

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self.state.amcl_x = msg.pose.pose.position.x
            self.state.amcl_y = msg.pose.pose.position.y

    def on_cmd_vel(self, msg: Twist):
        with self._lock:
            self.state.cmd_vx = msg.linear.x
            self.state.cmd_wz = msg.angular.z

    def on_odom(self, msg: Odometry):
        with self._lock:
            self.state.odom_vx = msg.twist.twist.linear.x
            self.state.odom_wz = msg.twist.twist.angular.z
            self.state.last_odom_time = datetime.now()

    def on_goal_pose(self, msg: PoseStamped):
        with self._lock:
            self.state.goal_x = msg.pose.position.x
            self.state.goal_y = msg.pose.position.y
            self.state.last_goal_time = datetime.now()

    def on_nav_status(self, msg: GoalStatusArray):
        with self._lock:
            if msg.status_list:
                latest = msg.status_list[-1].status
                self.state.nav_status_code = int(latest)
                self.state.nav_status_text = status_from_code(self.state.nav_status_code)
                self.state.last_status_time = datetime.now()

    def refresh_node_health(self):
        names = set(self.get_node_names())
        with self._lock:
            self.state.sim_ok = "clock_bridge" in names
            self.state.loc_ok = ("map_server" in names and "amcl" in names)
            self.state.nav_ok = ("bt_navigator" in names and "controller_server" in names)

    def infer_step(self) -> str:
        with self._lock:
            s = self.state
            moving = False
            if s.odom_vx is not None and abs(s.odom_vx) > 0.02:
                moving = True
            if s.odom_wz is not None and abs(s.odom_wz) > 0.08:
                moving = True

            if not s.sim_ok:
                return "STEP 1: Starting simulation"
            if not s.loc_ok:
                return "STEP 2: Starting localization"
            if not s.nav_ok:
                return "STEP 3: Starting Nav2"
            if s.amcl_x is None or s.amcl_y is None:
                return "STEP 4: Waiting initial pose (2D Pose Estimate)"
            if s.nav_status_text in ("EXECUTING", "ACCEPTED") or moving:
                return "STEP 5: Navigating to goal"
            if s.nav_status_text == "SUCCEEDED":
                return "STEP 6: Goal reached"
            if s.nav_status_text == "ABORTED":
                return "STEP 6: Navigation aborted"
            if s.goal_x is not None and s.goal_y is not None:
                return "STEP 5: Goal sent, waiting motion"
            return "STEP 5: Waiting goal (Nav2 Goal)"

    def render(self):
        self.refresh_node_health()
        step = self.infer_step()
        with self._lock:
            s = self.state
            goal_xy = f"x={fmt(s.goal_x)}, y={fmt(s.goal_y)}"
            now = datetime.now()

            _ = now

        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.write("==== TurtleBot4 Maze Loop Monitor (Subscriber Mode) ====\n")
        sys.stdout.write(f"World: {self.world_name}\n")
        sys.stdout.write(f"Map:   {self.map_path}\n\n")
        sys.stdout.write(f"Current Loop Step: {step}\n\n")
        sys.stdout.write("[Node Health]\n")
        sys.stdout.write(f"  simulation (/clock_bridge):         {'YES' if s.sim_ok else 'NO'}\n")
        sys.stdout.write(f"  localization (/map_server,/amcl):   {'YES' if s.loc_ok else 'NO'}\n")
        sys.stdout.write(f"  nav2 (/bt_navigator,/controller):   {'YES' if s.nav_ok else 'NO'}\n\n")
        sys.stdout.write("[Key Data]\n")
        sys.stdout.write(f"  amcl_pose:  x={fmt(s.amcl_x)}, y={fmt(s.amcl_y)}\n")
        sys.stdout.write(f"  cmd_vel:    vx={fmt(s.cmd_vx)}, wz={fmt(s.cmd_wz)}\n")
        sys.stdout.write(f"  odom_twist: vx={fmt(s.odom_vx)}, wz={fmt(s.odom_wz)}\n")
        sys.stdout.write(f"  goal_pose:  {goal_xy}\n")
        sys.stdout.write(f"  goal_state: {s.nav_status_text}\n\n")
        sys.stdout.write("Controls: press Esc in this terminal to stop all.\n")
        sys.stdout.flush()


def main():
    world_name = sys.argv[1] if len(sys.argv) > 1 else "maze"
    map_path = sys.argv[2] if len(sys.argv) > 2 else "unknown"

    rclpy.init()
    node = NavLoopMonitor(world_name, map_path)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok() and not node.should_stop():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.render()
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    node.request_stop()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
