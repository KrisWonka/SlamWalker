"""
SlamWalker Navigation launch: serial bridge + LiDAR + robot_state_publisher + Nav2 (with map).
Use this after you have a map to do autonomous navigation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_dir = get_package_share_directory('slamwalker_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'slamwalker.urdf.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_real.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'nav.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/arduino',
        description='Arduino serial port')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ldlidar',
        description='LiDAR serial port')
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model', default_value='LDLiDAR_LD19',
        description='LDLiDAR product name')
    map_arg = DeclareLaunchArgument(
        'map', description='Full path to map yaml file')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start rviz2 locally. Keep false on headless Jetson; '
                    'view RViz remotely from krix-mac instead.')

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
            'wheel_base': 0.3913,
            'ticks_per_meter': 15623.1,
            'right_encoder_scale': 1.0,
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )

    # Use the installed package node, NOT /tmp/scan_resampler.py — /tmp is
    # wiped on reboot, which silently kills /scan and leaves amcl with no
    # laser → never localizes → no map->odom tf → whole nav2 stack stuck.
    scan_resampler = Node(
        package='slamwalker_explore', executable='scan_resampler',
        name='scan_resampler', output='screen',
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

    def _launch_nav2(context, *args, **kwargs):
        map_path = os.path.expanduser(
            LaunchConfiguration('map').perform(context))
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': 'false',
                'params_file': nav2_params,
            }.items(),
        )]

    from launch.conditions import IfCondition
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    ld = LaunchDescription()
    ld.add_action(serial_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(lidar_model_arg)
    ld.add_action(map_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(serial_bridge)
    ld.add_action(ldlidar)
    ld.add_action(scan_resampler)
    ld.add_action(OpaqueFunction(function=_launch_nav2))
    ld.add_action(rviz2)
    return ld
