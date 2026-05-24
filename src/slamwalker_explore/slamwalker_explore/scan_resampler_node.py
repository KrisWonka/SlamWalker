#!/usr/bin/env python3
"""
Resample a variable-count LaserScan into a fixed-count one.

Subscribes:  /scan_raw  (LaserScan, variable beam count)
Publishes:   /scan      (LaserScan, fixed 'samples' count)

slam_toolbox / karto requires consistent beam count across scans;
LD19's driver emits variable counts per rotation, which makes karto
silently drop most scans ("LaserRangeScan contains X readings, expected Y").
This node forces every published /scan to a fixed grid.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Resampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')
        self.declare_parameter('samples', 360)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.samples = int(self.get_parameter('samples').value)
        in_t = self.get_parameter('input_topic').value
        out_t = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(LaserScan, out_t, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, in_t, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'resampling {in_t} -> {out_t} @ {self.samples} samples')

    def cb(self, msg: LaserScan):
        n_in = len(msg.ranges)
        if n_in < 2:
            return
        # Build fixed output grid from msg.angle_min to msg.angle_max
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.scan_time = msg.scan_time
        out.angle_increment = (msg.angle_max - msg.angle_min) / (self.samples - 1)
        out.time_increment = msg.scan_time / max(self.samples - 1, 1)
        # Sample by nearest input bin
        in_ranges = np.array(msg.ranges, dtype=np.float32)
        # source angle per input bin
        src_inc = (msg.angle_max - msg.angle_min) / max(n_in - 1, 1)
        out_ranges = np.full(self.samples, float('nan'), dtype=np.float32)
        for i in range(self.samples):
            a = msg.angle_min + i * out.angle_increment
            j = int(round((a - msg.angle_min) / src_inc))
            if 0 <= j < n_in:
                v = in_ranges[j]
                if math.isfinite(v) and msg.range_min <= v <= msg.range_max:
                    out_ranges[i] = v
        out.ranges = out_ranges.tolist()
        out.intensities = [0.0] * self.samples
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Resampler()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
