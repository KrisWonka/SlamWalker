#!/usr/bin/env python3
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


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


class LoopChainStream(Node):
    def __init__(self):
        super().__init__("loop_chain_stream")
        self.nav_ok = False
        self.local_plan_points = 0
        self.last_local_plan_ts: datetime | None = None
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0
        self.nav_status_code: int | None = None
        self.last_emitted = ""
        # Heartbeat cycle state (for visible loop progression).
        self.phase = 0
        self.cycle = 0

        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Path, "/local_plan", self.on_local_plan, default_qos)
        self.create_subscription(Path, "/controller_server/local_plan", self.on_local_plan, default_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, default_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, default_qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_nav_status, default_qos)
        # Read node graph periodically for DWB/nav availability.
        self.create_timer(0.2, self.on_timer)

        # Avoid stdout buffering to keep fast updates visible.
        sys.stdout.reconfigure(line_buffering=True)
        self.get_logger().info("loop-chain stream started")

    def on_local_plan(self, msg: Path):
        self.local_plan_points = len(msg.poses)
        self.last_local_plan_ts = datetime.now()
        # Every fresh local plan marks a new loop cycle.
        if self.nav_ok:
            self.cycle += 1
            # Show explicit return to DWB before entering local_path.
            self.phase = 1
            self.emit_chain_if_changed()
            self.phase = 2
        self.emit_chain_if_changed()

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        motor_active = abs(self.cmd_vx) > 0.01 or abs(self.cmd_wz) > 0.05
        if motor_active and self.phase < 3:
            self.phase = 3
        self.emit_chain_if_changed()

    def on_odom(self, msg: Odometry):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        robot_moves = abs(self.odom_vx) > 0.02 or abs(self.odom_wz) > 0.08
        if robot_moves and self.phase < 4:
            self.phase = 4
        self.emit_chain_if_changed()

    def on_nav_status(self, msg: GoalStatusArray):
        if msg.status_list:
            self.nav_status_code = int(msg.status_list[-1].status)
        self.emit_chain_if_changed()

    def on_timer(self):
        names = set(self.get_node_names())
        self.nav_ok = ("bt_navigator" in names and "controller_server" in names)
        if not self.nav_ok:
            self.phase = 0
        elif self.phase == 0:
            self.phase = 1
        self.emit_chain_if_changed()

    def infer_chain(self):
        now = datetime.now()
        local_path_ok = (
            self.last_local_plan_ts is not None
            and (now - self.last_local_plan_ts).total_seconds() < 1.5
            and self.local_plan_points > 0
        )
        motor_active = abs(self.cmd_vx) > 0.01 or abs(self.cmd_wz) > 0.05
        robot_moves = abs(self.odom_vx) > 0.02 or abs(self.odom_wz) > 0.08
        nav_state = status_from_code(self.nav_status_code)
        stop_done = nav_state == "SUCCEEDED" and not robot_moves and not motor_active

        # Keep a state-machine style "current step" so loop progression is visible.
        if stop_done:
            self.phase = 5
        elif self.phase == 2 and not local_path_ok:
            # If local path is stale, return to DWB wait state.
            self.phase = 1 if self.nav_ok else 0
        elif self.phase == 4 and not robot_moves and nav_state != "SUCCEEDED":
            # Robot stopped but goal not done yet -> waiting next DWB/local-path cycle.
            self.phase = 1 if self.nav_ok else 0

        phase_to_label = {
            0: "0/5 waiting_nav2",
            1: "1/5 dwb",
            2: "2/5 local_path",
            3: "3/5 motor_command",
            4: "4/5 robot_moves",
            5: "5/5 stop",
        }
        current = phase_to_label.get(self.phase, "0/5 waiting_nav2")

        return current, local_path_ok, motor_active, robot_moves, stop_done, nav_state

    def emit_chain_if_changed(self):
        current, local_path_ok, motor_active, robot_moves, stop_done, nav_state = self.infer_chain()
        line = (
            f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} | cycle={self.cycle} | current={current} | "
            f"dwb={'OK' if self.nav_ok else 'WAIT'} | "
            f"local_path={'OK' if local_path_ok else 'WAIT'}({self.local_plan_points}) | "
            f"motor={'ACTIVE' if motor_active else 'IDLE'}({self.cmd_vx:.3f},{self.cmd_wz:.3f}) | "
            f"robot_moves={'YES' if robot_moves else 'NO'}({self.odom_vx:.3f},{self.odom_wz:.3f}) | "
            f"stop={'YES' if stop_done else 'NO'} | nav={nav_state}"
        )
        if line != self.last_emitted:
            print(line)
            self.last_emitted = line


def main():
    rclpy.init()
    node = LoopChainStream()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
