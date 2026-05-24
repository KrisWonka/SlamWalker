# BNO055 IMU launch — standalone, can also be included from slamwalker_nav.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('slamwalker_bringup')
    default_params = PathJoinSubstitution(
        [pkg_bringup, 'config', 'bno055_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to BNO055 ROS params yaml',
        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
