"""
SlamWalker Phase 1: autonomous frontier exploration.

Stack:
  serial_bridge + ldlidar + robot_state_publisher  (hardware bringup)
  slam_toolbox (online mapping, publishes /map and map->odom)
  Nav2 (planner + controller + behavior + bt_navigator + velocity_smoother)
  frontier_explorer (this package: extracts frontiers, scores, sends goals)

Phase 1 terminates when no reachable frontier remains and auto-saves
~/walker_ws/maps/auto_map.yaml, which feeds Phase 2 (slamwalker_nav.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess


def generate_launch_description():
    bringup_dir = get_package_share_directory('slamwalker_bringup')
    explore_dir = get_package_share_directory('slamwalker_explore')

    urdf_file = os.path.join(bringup_dir, 'urdf', 'slamwalker.urdf.xacro')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(explore_dir, 'config', 'nav2_explore.yaml')
    explore_params = os.path.join(explore_dir, 'config', 'explore.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/arduino')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ldlidar')
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model', default_value='LDLiDAR_LD19')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true')
    start_explorer_arg = DeclareLaunchArgument(
        'start_explorer', default_value='true',
        description='set false to bringup hardware+SLAM+Nav2 only; start frontier_explorer manually later')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
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
            'range_min': 0.02,
            'range_max': 12.0,
        }],
    )

    scan_resampler = Node(
        package='slamwalker_explore', executable='scan_resampler',
        name='scan_resampler', output='screen',
    )
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    nav2_nodes = GroupAction([
        SetParameter(name='use_sim_time', value=False),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params]),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[nav2_params]),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[nav2_params],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen',
             parameters=[{
                 'autostart': LaunchConfiguration('autostart'),
                 'node_names': [
                     'controller_server',
                     'planner_server',
                     'behavior_server',
                     'bt_navigator',
                     'waypoint_follower',
                     'velocity_smoother',
                 ],
             }]),
    ])

    from launch.conditions import IfCondition
    frontier_explorer = Node(
        package='slamwalker_explore',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[explore_params],
        condition=IfCondition(LaunchConfiguration('start_explorer')),
    )

    ld = LaunchDescription()
    ld.add_action(serial_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(lidar_model_arg)
    ld.add_action(autostart_arg)
    ld.add_action(start_explorer_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(serial_bridge)
    ld.add_action(ldlidar)
    ld.add_action(scan_resampler)
    ld.add_action(slam_toolbox)
    ld.add_action(nav2_nodes)
    ld.add_action(frontier_explorer)
    return ld
