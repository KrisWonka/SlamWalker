"""
SlamWalker SLAM launch: serial bridge + LiDAR + robot_state_publisher + slam_toolbox + RViz.
Use this to build a map of a new environment.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_dir = get_package_share_directory('slamwalker_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'slamwalker.urdf.xacro')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/arduino',
        description='Arduino serial port')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ldlidar',
        description='LiDAR serial port')
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model', default_value='LDLiDAR_LD19',
        description='LDLiDAR product name (LDLiDAR_LD06 / LDLiDAR_LD19 / ...)')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baud', default_value='230400',
        description='LiDAR serial baud rate')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': False}],
        output='screen',
    )

    serial_bridge = Node(
        package='slamwalker_bridge',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        parameters=[{
            'port': LaunchConfiguration('serial_port'),
            'baud': 115200,
            'rate_hz': 20.0,
            'wheel_base': 0.40,
            'ticks_per_meter': 16400.0,
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )

    ldlidar = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher',
        output='screen',
        parameters=[{
            'product_name': LaunchConfiguration('lidar_model'),
            'laser_scan_topic_name': 'scan_raw',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'base_laser',
            'port_name': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
            'range_min': 0.02,
            'range_max': 12.0,
        }],
    )

    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    scan_resampler = ExecuteProcess(
        cmd=['python3', '/tmp/scan_resampler.py'],
        output='screen',
    )
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    rviz_config = os.path.join(pkg_dir, 'config', 'slam.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(serial_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(lidar_model_arg)
    ld.add_action(lidar_baud_arg)
    ld.add_action(rviz_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(serial_bridge)
    ld.add_action(ldlidar)
    ld.add_action(scan_resampler)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz_node)
    return ld
