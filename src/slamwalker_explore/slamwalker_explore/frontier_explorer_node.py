#!/usr/bin/env python3
"""
SlamWalker locomotion-aware frontier explorer (Phase 1).

Pipeline (lane 3 of phase1_autonomous_exploration_clean.png):
  /map (slam_toolbox) ---> extract frontier cells
                       --> cluster into candidate goals
                       --> score with U = alpha*distance + beta*turn + gamma*info_gain
                       --> send NavigateToPose action to Nav2
                       --> repeat until no reachable frontier
                       --> save auto_map.yaml (Phase 2 input)

The "beta*turn" term is the locomotion-aware contribution: the walking
mechanism pays a disproportionate cost for in-place rotation compared to
a differential-wheeled base (ticks_per_meter is 2.14x the wheel-radius
prediction; PWM dead-zone is 230/255). The classical Yamauchi 1997 /
explore_lite utility uses only distance and info gain.

Set beta=0 to recover an explore_lite-style baseline for ablation.
"""
import math
import os
import subprocess
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros


# Occupancy grid cell semantics
FREE = 0
UNKNOWN = -1
# anything > OCC_THRESH is treated as obstacle
OCC_THRESH = 50


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- locomotion-aware utility weights ----
        self.declare_parameter('alpha_distance', 1.0)
        self.declare_parameter('beta_turn', 2.5)      # walking-aware: penalize big turns
        self.declare_parameter('gamma_info', 5.0)
        # ---- geometry ----
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('min_frontier_size', 3)       # min cluster cells
        self.declare_parameter('min_info_gain', 80)          # min unknown cells around a frontier to be worth visiting
        self.declare_parameter('info_gain_radius_m', 1.0)    # unknown cells within this radius count as info gain
        # ---- planning ----
        self.declare_parameter('replan_period_s', 3.0)
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('progress_min_m', 0.10)
        self.declare_parameter('progress_window_s', 12.0)
        # ---- termination / output ----
        self.declare_parameter('max_failed_goals', 5)
        self.declare_parameter('map_save_path', os.path.expanduser('~/walker_ws/maps/auto_map'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        # ---- preemptive goal switching (with hysteresis to avoid thrashing) ----
        # While driving to a goal, if a re-scored candidate is much better,
        # switch to it. Requires: (1) current goal committed >= min_commit_s,
        # (2) new utility >= current * preempt_factor, (3) new goal far enough
        # from current goal to be worth a re-route.
        self.declare_parameter('preempt_factor', 1.5)
        self.declare_parameter('preempt_min_commit_s', 5.0)
        self.declare_parameter('preempt_min_move_m', 0.5)

        self.alpha = self.get_parameter('alpha_distance').value
        self.beta = self.get_parameter('beta_turn').value
        self.gamma = self.get_parameter('gamma_info').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.min_info_gain = int(self.get_parameter('min_info_gain').value)
        self.info_radius = self.get_parameter('info_gain_radius_m').value
        self.replan_period = self.get_parameter('replan_period_s').value
        self.goal_timeout = self.get_parameter('goal_timeout_s').value
        self.progress_min = self.get_parameter('progress_min_m').value
        self.progress_window = self.get_parameter('progress_window_s').value
        self.max_failed = int(self.get_parameter('max_failed_goals').value)
        self.map_save_path = self.get_parameter('map_save_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.preempt_factor = self.get_parameter('preempt_factor').value
        self.preempt_min_commit = self.get_parameter('preempt_min_commit_s').value
        self.preempt_min_move = self.get_parameter('preempt_min_move_m').value

        self.get_logger().info(
            f'utility weights: alpha={self.alpha} beta={self.beta} gamma={self.gamma}')

        # State
        self.latest_map: OccupancyGrid | None = None
        self.map_lock = threading.Lock()
        self.failed_goals = 0
        self.current_goal_handle = None
        self.goal_active = False
        self.goal_start_pose = None
        self.goal_start_time = None
        self.shutdown_requested = False
        self._current_goal_xy = None   # (x, y) of the goal we're driving to
        self._current_goal_U = None    # utility score of current goal
        self._goal_epoch = 0           # bumped each _send_goal; stale callbacks ignored

        # ROS interfaces
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontiers', 1)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_timer(self.replan_period, self._tick)
        self.get_logger().info('FrontierExplorer up; waiting for /map and Nav2 action server...')

    # ----------------------------- callbacks ------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        with self.map_lock:
            self.latest_map = msg

    # ------------------------------ main tick -----------------------------

    def _tick(self):
        if self.shutdown_requested:
            return
        with self.map_lock:
            grid = self.latest_map
        if grid is None:
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().info('no TF map->robot yet, waiting...', throttle_duration_sec=3.0)
            return

        candidates = self._find_frontier_candidates(grid)
        self._publish_markers(grid, candidates)

        if self.goal_active:
            self._check_goal_progress(robot_xy)
            self._maybe_preempt(candidates, robot_xy)
            return

        if not candidates:
            self.get_logger().warn('No frontier candidates remain.')
            self._maybe_terminate(reason='no frontiers found')
            return

        robot_yaw = self._get_robot_yaw()
        scored = self._score_candidates(candidates, robot_xy, robot_yaw, grid)
        if not scored:
            self._maybe_terminate(reason='no scorable candidates')
            return

        best = scored[0]
        self.get_logger().info(
            f'best frontier: xy=({best["x"]:.2f},{best["y"]:.2f}) '
            f'U={best["U"]:.2f} d={best["d"]:.2f} turn={math.degrees(best["turn"]):.0f}deg '
            f'info={best["info"]} cluster={best["size"]}')
        self._send_goal(best['x'], best['y'], best['U'])

    def _maybe_preempt(self, candidates, robot_xy):
        """While driving to a goal, switch to a markedly better candidate.
        Three hysteresis gates prevent goal thrashing / circling:
          1) current goal committed >= preempt_min_commit seconds
          2) new utility >= current utility * preempt_factor
          3) new goal at least preempt_min_move from current goal
        """
        if (self.goal_start_time is None or self._current_goal_U is None
                or self._current_goal_xy is None):
            return
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
        if elapsed < self.preempt_min_commit:
            return
        if not candidates:
            return
        robot_yaw = self._get_robot_yaw()
        with self.map_lock:
            grid = self.latest_map
        if grid is None:
            return
        scored = self._score_candidates(candidates, robot_xy, robot_yaw, grid)
        if not scored:
            return
        best = scored[0]
        # gate 3: new goal must be meaningfully different from current
        dx = best['x'] - self._current_goal_xy[0]
        dy = best['y'] - self._current_goal_xy[1]
        if math.hypot(dx, dy) < self.preempt_min_move:
            return
        # gate 2: must be significantly better
        if best['U'] <= self._current_goal_U * self.preempt_factor:
            return
        self.get_logger().info(
            f'PREEMPT: new U={best["U"]:.0f} > current U={self._current_goal_U:.0f} '
            f'x{self.preempt_factor} after {elapsed:.0f}s; switching to '
            f'({best["x"]:.2f},{best["y"]:.2f})')
        # Directly send the new goal. Nav2's single-goal action server aborts
        # the old one; the old goal's result callback carries a stale epoch
        # and is ignored, so no spurious failed_goals++ or state clobber.
        self._send_goal(best['x'], best['y'], best['U'])

    # ----------------------- frontier extraction --------------------------

    def _find_frontier_candidates(self, grid: OccupancyGrid):
        w = grid.info.width
        h = grid.info.height
        data = np.asarray(grid.data, dtype=np.int8).reshape((h, w))

        # frontier cell = FREE that has at least one UNKNOWN 4-neighbor
        free = data == FREE
        unknown = data == UNKNOWN

        unk_n = np.zeros_like(unknown)
        unk_n[1:, :]   |= unknown[:-1, :]
        unk_n[:-1, :]  |= unknown[1:, :]
        unk_n[:, 1:]   |= unknown[:, :-1]
        unk_n[:, :-1]  |= unknown[:, 1:]

        frontier_mask = free & unk_n

        # BFS-cluster frontier cells (4-connectivity)
        visited = np.zeros_like(frontier_mask)
        clusters = []
        idxs = np.argwhere(frontier_mask)
        idx_set = {(r, c) for r, c in idxs}

        for (r0, c0) in idxs:
            if visited[r0, c0]:
                continue
            stack = deque([(r0, c0)])
            visited[r0, c0] = True
            cells = []
            while stack:
                r, c = stack.pop()
                cells.append((r, c))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in idx_set and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            if len(cells) >= self.min_frontier_size:
                clusters.append(cells)

        # convert clusters to world-frame candidate goals (centroid)
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        candidates = []
        for cells in clusters:
            rs = np.array([c[0] for c in cells])
            cs = np.array([c[1] for c in cells])
            cr = rs.mean()
            cc = cs.mean()
            wx = ox + (cc + 0.5) * res
            wy = oy + (cr + 0.5) * res
            # Keep only if centroid is free AND has robot_radius clearance
            # from any obstacle. Without the clearance check, frontier picks
            # goals right next to walls, which costmap inflation then turns
            # into "unreachable in pure-free space" → planner fails.
            ri = int(round(cr))
            ci = int(round(cc))
            if (0 <= ri < h and 0 <= ci < w and data[ri, ci] == FREE
                    and self._has_obstacle_clearance(data, ri, ci, res)):
                candidates.append({
                    'x': wx, 'y': wy,
                    'cells': cells, 'size': len(cells),
                    'row': ri, 'col': ci,
                })
        return candidates

    def _has_obstacle_clearance(self, data, row: int, col: int, res: float) -> bool:
        """Reject frontier centroids whose robot-radius disk overlaps occupied cells."""
        radius_cells = max(1, int(math.ceil(self.robot_radius / res)))
        h, w = data.shape
        r0 = max(0, row - radius_cells)
        r1 = min(h, row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(w, col + radius_cells + 1)
        rr, cc = np.ogrid[r0:r1, c0:c1]
        mask = (rr - row) ** 2 + (cc - col) ** 2 <= radius_cells ** 2
        patch = data[r0:r1, c0:c1]
        return not bool(np.any(patch[mask] > OCC_THRESH))

    # ----------------------- utility scoring ------------------------------

    def _score_candidates(self, candidates, robot_xy, robot_yaw, grid: OccupancyGrid):
        if not candidates:
            return []
        rx, ry = robot_xy
        res = grid.info.resolution
        w = grid.info.width
        h = grid.info.height
        data = np.asarray(grid.data, dtype=np.int8).reshape((h, w))

        info_r_cells = max(1, int(round(self.info_radius / res)))

        scored = []
        for c in candidates:
            dx = c['x'] - rx
            dy = c['y'] - ry
            d = math.hypot(dx, dy)
            if d < 1e-3:
                continue
            target_yaw = math.atan2(dy, dx)
            turn = abs(self._wrap(target_yaw - robot_yaw))

            r0 = max(0, c['row'] - info_r_cells)
            r1 = min(h, c['row'] + info_r_cells + 1)
            c0 = max(0, c['col'] - info_r_cells)
            c1 = min(w, c['col'] + info_r_cells + 1)
            patch = data[r0:r1, c0:c1]
            info = int(np.count_nonzero(patch == UNKNOWN))

            if info < self.min_info_gain:
                continue
            # higher U = better
            U = (self.gamma * info) - (self.alpha * d) - (self.beta * turn)
            scored.append({**c, 'd': d, 'turn': turn, 'info': info, 'U': U,
                           'target_yaw': target_yaw})

        scored.sort(key=lambda s: s['U'], reverse=True)
        return scored

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    # ----------------------- Nav2 goal handling ---------------------------

    def _send_goal(self, x, y, U=None):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 NavigateToPose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # yaw free; let Nav2 plan

        self.goal_active = True
        self.goal_start_time = self.get_clock().now()
        self.goal_start_pose = self._get_robot_xy()
        self._current_goal_xy = (x, y)
        self._current_goal_U = U
        self._goal_epoch += 1
        epoch = self._goal_epoch

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, epoch))

    def _on_goal_response(self, future, epoch):
        if epoch != self._goal_epoch:
            return  # superseded by a newer goal (preemption)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            self._goal_done(success=False)
            return
        self.current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_goal_result(f, epoch))

    def _on_goal_result(self, future, epoch):
        if epoch != self._goal_epoch:
            return  # stale result from a preempted goal; ignore
        status = future.result().status  # action_msgs/GoalStatus
        # 4 == SUCCEEDED
        success = (status == 4)
        if success:
            self.get_logger().info('goal SUCCEEDED')
            self.failed_goals = 0
        else:
            self.get_logger().warn(f'goal ended with status={status}')
            self.failed_goals += 1
        self._goal_done(success=success)

    def _check_goal_progress(self, robot_xy):
        if self.goal_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
        if elapsed > self.goal_timeout:
            self.get_logger().warn(f'goal timeout after {elapsed:.1f}s, cancelling')
            self._cancel_current_goal()
            return
        if elapsed > self.progress_window and self.goal_start_pose is not None:
            dx = robot_xy[0] - self.goal_start_pose[0]
            dy = robot_xy[1] - self.goal_start_pose[1]
            if math.hypot(dx, dy) < self.progress_min:
                self.get_logger().warn(
                    f'no progress (<{self.progress_min}m in {elapsed:.0f}s), cancelling')
                self._cancel_current_goal()

    def _cancel_current_goal(self):
        if self.current_goal_handle is None:
            self._goal_done(success=False)
            return
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: self._goal_done(success=False))

    def _goal_done(self, success: bool):
        if not self.goal_active:
            return  # idempotent: ignore double callbacks (cancel + result)
        self.goal_active = False
        self.current_goal_handle = None
        self.goal_start_pose = None
        self.goal_start_time = None
        self._current_goal_xy = None
        self._current_goal_U = None
        if not success and self.failed_goals >= self.max_failed:
            self._maybe_terminate(reason=f'{self.failed_goals} consecutive failed goals')

    # ----------------------- termination ----------------------------------

    def _maybe_terminate(self, reason: str):
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.get_logger().warn(f'TERMINATION: {reason}. Saving map to {self.map_save_path}.yaml')
        try:
            os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)
            r = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self.map_save_path],
                capture_output=True, text=True, timeout=30,
            )
            self.get_logger().info(f'map_saver stdout: {r.stdout.strip()}')
            if r.returncode != 0:
                self.get_logger().error(f'map_saver failed: {r.stderr.strip()}')
            else:
                self.get_logger().info(
                    f'Phase 1 complete. {self.map_save_path}.yaml is Phase 2 input.')
        except Exception as e:
            self.get_logger().error(f'map_saver exception: {e}')

    # ----------------------- TF helpers -----------------------------------

    def _get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _get_robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return 0.0
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ----------------------- visualization --------------------------------

    def _publish_markers(self, grid: OccupancyGrid, candidates):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = self.map_frame
        ma.markers.append(clear)
        for i, c in enumerate(candidates):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontier_centroids'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c['x']
            m.pose.position.y = c['y']
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.1
            m.color.a = 0.9
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C: cancelling Nav2 goal + saving map')
        try:
            if node.current_goal_handle is not None:
                cancel_future = node.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
        except Exception as e:
            node.get_logger().warn(f'cancel failed: {e}')
        # Save the current map just like the auto-termination path
        try:
            node._maybe_terminate(reason='manual stop (Ctrl+C)')
        except Exception as e:
            node.get_logger().warn(f'map save failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
