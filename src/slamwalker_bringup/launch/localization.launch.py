"""
Localization stack: BNO055 IMU + robot_localization EKF.

Subscribes:
  /odom            (from slamwalker_bridge — wheel-encoder odometry)
  /bno055/imu      (from BNO055 driver — 6-axis on-chip fused orientation)
Publishes:
  odom -> base_footprint TF  (replaces serial_bridge's TF — set publish_tf:=false there)
  /odometry/filtered
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slamwalker_bringup')
    bno055_launch = os.path.join(pkg, 'launch', 'bno055.launch.py')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bno055_launch)
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params],
        ),
    ])
