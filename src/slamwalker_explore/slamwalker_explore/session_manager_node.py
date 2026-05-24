#!/usr/bin/env python3
"""
Session manager: orchestrates Phase 1 / Phase 2 stack transitions.

Services (all std_srvs/srv/Trigger):
  /walker/start_frontier     — kill any current stack, start Phase 1 (slam+nav+frontier)
  /walker/finish_to_phase2   — save current SLAM map, switch to Phase 2 (amcl+map_server+nav)
  /walker/load_map           — load the map at parameter `map_path` into Phase 2
  /walker/reset_pose         — publish initial pose (0,0,0) to AMCL

Parameters:
  map_path           default '/home/kris_nano/walker_ws/maps/auto_map.yaml'
  cyclone_uri        default 'file:///home/kris_nano/cyclonedds.xml'
  workspace_setup    default '/home/kris_nano/walker_ws/install/setup.bash'
"""
import fcntl
import os
import signal
import subprocess
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped


KILL_TARGETS = [
    'ros2 launch', 'slam_toolbox', 'serial_bridge_node', 'ldlidar_ros2_node',
    'controller_server', 'bt_navigator', 'planner_server', 'behavior_server',
    'velocity_smoother', 'lifecycle_manager', 'waypoint_follower',
    'frontier_explorer', 'robot_state_publisher', 'scan_resampler',
    'nav2_amcl', 'nav2_map_server',
]


class SessionManager(Node):
    def __init__(self):
        super().__init__('session_manager')
        self.declare_parameter(
            'map_path', '/home/kris_nano/walker_ws/maps/auto_map.yaml')
        self.declare_parameter(
            'cyclone_uri', 'file:///home/kris_nano/cyclonedds.xml')
        self.declare_parameter(
            'workspace_setup', '/home/kris_nano/walker_ws/install/setup.bash')
        self.declare_parameter('ros_domain_id', '0')

        self._proc = None
        self._mode = 'idle'

        self.create_service(Trigger, '/walker/start_bringup', self._cb_start_bringup)
        self.create_service(Trigger, '/walker/start_frontier', self._cb_start_frontier)
        self.create_service(Trigger, '/walker/stop_frontier', self._cb_stop_frontier)
        self.create_service(Trigger, '/walker/finish_to_phase2', self._cb_finish_phase2)
        self.create_service(Trigger, '/walker/load_map', self._cb_load_map)
        self.create_service(Trigger, '/walker/reset_pose', self._cb_reset_pose)
        self.create_service(Trigger, '/walker/stop', self._cb_stop)

        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Track latest SLAM pose so finish_to_phase2 can hand it to AMCL
        self._last_slam_pose = None  # PoseWithCovarianceStamped
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose',
            self._slam_pose_cb, 10)

        self.get_logger().info('session_manager up.')

    # ---------------- helpers ------------------------------------------

    def _env(self):
        env = os.environ.copy()
        env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        env['CYCLONEDDS_URI'] = self.get_parameter('cyclone_uri').value
        env['ROS_DOMAIN_ID'] = self.get_parameter('ros_domain_id').value
        return env

    def _count_stack_procs(self) -> int:
        """Count surviving stack procs matched by any KILL_TARGETS pattern.
        Excludes ourselves and our parent shell."""
        pids = set()
        for t in KILL_TARGETS:
            r = subprocess.run(['pgrep', '-f', t],
                               capture_output=True, text=True, check=False)
            for line in r.stdout.strip().splitlines():
                line = line.strip()
                if line.isdigit():
                    pids.add(int(line))
        pids.discard(os.getpid())
        pids.discard(os.getppid())
        return len(pids)

    def _kill_stack(self) -> bool:
        """Tear down any running stack. Verifies pgrep finds 0 stack procs
        before returning. Up to 3 SIGKILL rounds. Returns True if clean,
        False if survivors remain (caller can surface as service error).

        Why: SIGKILL-once + sleep(3) wasn't enough — ros2 launch's grandchildren
        run in independent process groups and survived single-pass pkill, so
        repeated start_bringup accumulated 2x/3x of every node, all racing
        on /scan and /tf.
        """
        self.get_logger().info('killing existing stack...')

        # Fast path: kill our tracked launch process group
        if self._proc is not None:
            try:
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(0.5)
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            self._proc = None

        # SIGTERM round — lets ROS unwind DDS endpoints cleanly
        for t in KILL_TARGETS:
            subprocess.run(['pkill', '-15', '-f', t],
                           check=False, capture_output=True)
        time.sleep(1.5)

        # Up to 3 SIGKILL rounds, verifying after each
        for attempt in range(3):
            survivors = self._count_stack_procs()
            if survivors == 0:
                self.get_logger().info(
                    f'stack clean after {attempt + 1} pass(es); '
                    'waiting for DDS endpoints to release...')
                time.sleep(2.0)
                final = self._count_stack_procs()
                if final == 0:
                    return True
                self.get_logger().warn(
                    f'{final} procs respawned during DDS settle; '
                    'continuing to next attempt')
                continue
            self.get_logger().warn(
                f'attempt {attempt + 1}: {survivors} stack procs alive; SIGKILL')
            for t in KILL_TARGETS:
                subprocess.run(['pkill', '-9', '-f', t],
                               check=False, capture_output=True)
            time.sleep(1.5)

        final = self._count_stack_procs()
        if final > 0:
            self.get_logger().error(
                f'_kill_stack FAILED: {final} survivors after 3 rounds')
            return False
        time.sleep(2.0)
        return True

    def _launch(self, ros2_launch_cmd: str) -> int:
        env = self._env()
        ws = self.get_parameter('workspace_setup').value
        full = (f'source /opt/ros/humble/setup.bash && '
                f'source {ws} && '
                f'ros2 launch {ros2_launch_cmd}')
        self._proc = subprocess.Popen(
            ['bash', '-c', full],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        return self._proc.pid

    # ---------------- service callbacks --------------------------------

    def _cb_start_bringup(self, req, res):
        """Bring up hardware + SLAM + Nav2 (no frontier_explorer)."""
        if not self._kill_stack():
            res.success = False
            res.message = (
                '_kill_stack failed: stack procs survived 3 SIGKILL rounds. '
                'Refusing to launch (would create duplicate instances). '
                'SSH in and pkill manually, then retry.')
            self.get_logger().error(res.message)
            return res
        pid = self._launch(
            'slamwalker_explore slamwalker_explore.launch.py start_explorer:=false')
        self._mode = 'bringup'
        res.success = True
        res.message = f'bringup launched (pid={pid})'
        self.get_logger().info(res.message)
        return res

    def _cb_start_frontier(self, req, res):
        """Start frontier_explorer only. Requires bringup running."""
        # Kill any old frontier_explorer first
        subprocess.run(['pkill', '-9', '-f', 'frontier_explorer'],
                       check=False, capture_output=True)
        time.sleep(0.5)
        env = self._env()
        # Re-activate controller_server in case last EMERGENCY STOP
        # deactivated it. No-op if already active.
        subprocess.run(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'timeout 5 ros2 lifecycle set /controller_server activate '
             '>/dev/null 2>&1; true'],
            env=env, check=False, timeout=8)
        ws = self.get_parameter('workspace_setup').value
        explore_yaml = '/home/kris_nano/walker_ws/install/slamwalker_explore/share/slamwalker_explore/config/explore.yaml'
        full = (f'source /opt/ros/humble/setup.bash && '
                f'source {ws} && '
                f'ros2 run slamwalker_explore frontier_explorer --ros-args '
                f'--params-file {explore_yaml}')
        proc = subprocess.Popen(['bash', '-c', full],
                                env=env,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                preexec_fn=os.setsid)
        self._mode = 'frontier'
        res.success = True
        res.message = f'frontier_explorer started (pid={proc.pid})'
        self.get_logger().info(res.message)
        return res

    def _cb_stop_frontier(self, req, res):
        subprocess.run(['pkill', '-9', '-f', 'frontier_explorer'],
                       check=False, capture_output=True)
        self._mode = 'bringup'
        res.success = True
        res.message = 'frontier_explorer killed; bringup still running'
        self.get_logger().info(res.message)
        return res

    def _cb_finish_phase2(self, req, res):
        # 1) capture robot pose in map frame BEFORE killing slam_toolbox
        captured_pose = self._last_slam_pose
        if captured_pose is not None:
            self.get_logger().info(
                f'captured robot pose: ({captured_pose.pose.pose.position.x:.2f},'
                f' {captured_pose.pose.pose.position.y:.2f}) for AMCL seed')
        else:
            self.get_logger().warn('no /pose received from slam_toolbox; AMCL will need manual 2D Pose Estimate')

        # 2) save map (drop .yaml suffix; map_saver appends)
        env = self._env()
        save_path = self.get_parameter('map_path').value.replace('.yaml', '')
        self.get_logger().info(f'saving SLAM map -> {save_path}.yaml')
        try:
            saved = subprocess.run(
                ['bash', '-c',
                 f'source /opt/ros/humble/setup.bash && '
                 f'ros2 run nav2_map_server map_saver_cli -f {save_path}'],
                env=env, timeout=30, capture_output=True, text=True)
            self.get_logger().info(f'map_saver rc={saved.returncode}')
        except Exception as e:
            res.success = False
            res.message = f'map save failed: {e}'
            self.get_logger().error(res.message)
            return res

        self._kill_stack()
        pid = self._launch(
            f'slamwalker_bringup slamwalker_nav.launch.py '
            f'map:={self.get_parameter("map_path").value}')
        self._mode = 'phase2'

        # 3) Seed AMCL with captured pose once Phase 2 stack is up
        if captured_pose is not None:
            threading.Thread(
                target=self._seed_amcl_pose,
                args=(captured_pose,),
                daemon=True).start()

        res.success = True
        res.message = f'phase2 started with {save_path}.yaml (pid={pid}); will seed AMCL at last SLAM pose'
        self.get_logger().info(res.message)
        return res

    def _seed_amcl_pose(self, pose: PoseWithCovarianceStamped):
        """Wait for AMCL to come up after Phase 2 launch, then publish initial pose."""
        # Phase 2 launch takes ~10s to bring up Nav2 + AMCL
        time.sleep(12.0)
        # Refresh timestamp so AMCL accepts it
        pose.header.stamp = self.get_clock().now().to_msg()
        # Publish a few times in case the first publish hits before AMCL subscribed
        for _ in range(8):
            self._initialpose_pub.publish(pose)
            time.sleep(0.3)
        self.get_logger().info('seeded AMCL initial pose from last SLAM pose')

    def _cb_load_map(self, req, res):
        self._kill_stack()
        pid = self._launch(
            f'slamwalker_bringup slamwalker_nav.launch.py '
            f'map:={self.get_parameter("map_path").value}')
        self._mode = 'phase2'
        res.success = True
        res.message = f'phase2 with {self.get_parameter("map_path").value} (pid={pid})'
        self.get_logger().info(res.message)
        return res

    def _cb_stop(self, req, res):
        """Emergency stop.

        Previous version sent `ros2 action send_goal /navigate_to_pose {}`
        which is NOT a cancel — it dispatches a NEW goal at (0,0,0). And
        spamming /cmd_vel is futile because velocity_smoother keeps
        republishing non-zero from cmd_vel_nav at 20Hz.

        New approach:
        1) Kill frontier_explorer  (stops generating goals)
        2) Lifecycle-deactivate controller_server  (stops cmd_vel_nav output)
           → velocity_smoother sees no input for 0.2s, then publishes 0
        3) Spam zero /cmd_vel briefly as last-resort safety net
        Mode → 'stopped'; Start Frontier will re-activate controller_server.
        """
        subprocess.run(['pkill', '-9', '-f',
                        'slamwalker_explore/lib/slamwalker_explore/frontier_explorer'],
                       check=False, capture_output=True)
        env = self._env()
        # Deactivate controller_server (sync, so we know when cmd_vel stops)
        subprocess.run(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'timeout 5 ros2 lifecycle set /controller_server deactivate '
             '>/dev/null 2>&1; true'],
            env=env, check=False, timeout=8)
        # Last-resort safety: publish zero /cmd_vel at ~20Hz for 3s
        subprocess.Popen(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'for i in $(seq 1 60); do '
             '  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
             '  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1; '
             '  sleep 0.05; '
             'done'],
            env=env)
        self._mode = 'stopped'
        res.success = True
        res.message = ('emergency stop: frontier killed, controller_server '
                       'deactivated, cmd_vel zeroed. Start Frontier will '
                       're-activate controller automatically.')
        self.get_logger().warn(res.message)
        return res

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Cache latest pose published by slam_toolbox (robot in map frame).
        # Used to seed AMCL on Phase 2 startup.
        self._last_slam_pose = msg

    def _cb_reset_pose(self, req, res):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        for _ in range(5):
            self._initialpose_pub.publish(msg)
            time.sleep(0.1)
        res.success = True
        res.message = 'initialpose published (0,0,0)'
        self.get_logger().info(res.message)
        return res


SINGLETON_LOCK_PATH = '/tmp/walker_session_manager.lock'


def _acquire_singleton_lock():
    """Refuse to start if another session_manager is already running.
    Without this, multiple session_manager instances each respond to the same
    /walker/* services and one user request triggers N parallel _kill_stack +
    _launch chains -> duplicate ros2 launch trees -> 2x/3x of every node.
    """
    lock_file = open(SINGLETON_LOCK_PATH, 'w')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.stderr.write(
            f'FATAL: another session_manager already holds {SINGLETON_LOCK_PATH}. '
            f'Refusing to start (would cause duplicate service responders). '
            f'pkill -9 -f session_manager and retry.\n')
        sys.exit(1)
    lock_file.write(f'{os.getpid()}\n')
    lock_file.flush()
    return lock_file  # caller must keep ref so the lock survives


def main():
    _lock = _acquire_singleton_lock()  # noqa: F841 — kept alive for lock lifetime
    rclpy.init()
    n = SessionManager()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
