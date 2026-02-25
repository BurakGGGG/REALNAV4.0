"""
Adım Adım SLAM Launch Dosyası (Nav2 OLMADAN)
STM32 + RPLidar A2M12 + SLAM Toolbox

Sadece LiDAR ve SLAM test etmek için.
Nav2 ve Explorer yok. Önce bunun çalıştığını doğrulayacağız.

Kullanım (Raspi'de):
ros2 launch my_robot_bringup real_robot_lidar_slam.launch.py \
    serial_port:=/dev/ttyAMA0 \
    lidar_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_slam = FindPackageShare("slam_toolbox")
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value=TextSubstitution(text="/dev/ttyAMA0"),
        description="STM32 Serial port (Pi 5 UART)"
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value=TextSubstitution(text="115200"),
        description="Serial baud rate for STM32"
    )
    
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="RPLIDAR A2M12 USB port"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time (false for real robot)"
    )
    
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
        description="SLAM params yaml"
    )
    
    # Launch Configurations
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    lidar_port = LaunchConfiguration("lidar_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    
    # ---------- Include: real_robot_control_bringup.launch.py ----------
    # ROS2 Control, robot_state_publisher, controllers
    real_robot_control_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, "launch", "real_robot_control_bringup.launch.py"])
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    
    # ---------- LIDAR Node (RPLIDAR A2M12) ----------
    # start_rplidar.sh: arka planda başlatıp /scan kontrol eder, retry yapar
    rplidar_script = PathJoinSubstitution([pkg_bringup, "scripts", "start_rplidar.sh"])
    lidar_node = ExecuteProcess(
        cmd=['bash', rplidar_script, lidar_port],
        output='screen',
    )
    
    # ---------- SLAM Toolbox ----------
    slam_toolbox_launch = TimerAction(
        period=3.0,  # 3 saniye bekle (LiDAR hazır olsun)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": use_sim_time,
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_lidar_port,
        declare_use_sim_time,
        declare_slam_params,
        
        real_robot_control_bringup,  # 0s - ROS2 Control + TF
        lidar_node,                  # 1s - RPLIDAR A2M12
        slam_toolbox_launch,         # 3s - SLAM Toolbox
        # Nav2 ve Explorer YOK - adım adım test
    ])
