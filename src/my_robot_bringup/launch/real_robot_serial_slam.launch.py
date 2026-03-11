"""
Gercek Robot Serial + RPLidar + SLAM launch dosyasi.

Bu launch dosyasi:
1. real_robot_serial_bringup.launch.py include eder
2. RPLidar node'unu baslatir
3. SLAM Toolbox async node'unu baslatir
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    lidar_frame = LaunchConfiguration("lidar_frame")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    slam_params_file = LaunchConfiguration("slam_params_file")

    default_slam_params = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config",
        "mapper_params_online_async.yaml",
    )

    serial_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_bringup"),
                "launch",
                "real_robot_serial_bringup.launch.py",
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rplidar = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                parameters=[{
                    "serial_port": lidar_port,
                    "serial_baudrate": lidar_baud,
                    "frame_id": lidar_frame,
                    "inverted": inverted,
                    "angle_compensate": angle_compensate,
                    "use_sim_time": use_sim_time,
                }],
                remappings=[("scan", "/scan")],
            )
        ],
    )

    slam = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("scan", "/scan")],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("serial_port", default_value=TextSubstitution(text="/dev/ttyAMA0")),
        DeclareLaunchArgument("baud_rate", default_value=TextSubstitution(text="115200")),
        DeclareLaunchArgument("lidar_port", default_value=TextSubstitution(text="/dev/ttyUSB0")),
        DeclareLaunchArgument("lidar_baud", default_value=TextSubstitution(text="115200")),
        DeclareLaunchArgument("lidar_frame", default_value=TextSubstitution(text="laser_link")),
        DeclareLaunchArgument("inverted", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("angle_compensate", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("slam_params_file", default_value=TextSubstitution(text=default_slam_params)),
        serial_bringup,
        rplidar,
        slam,
    ])
