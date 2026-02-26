"""
Lightweight SLAM Launch — ros2_control OLMADAN
================================================
- robot_state_publisher (URDF → TF)
- RPLIDAR A2M12 (start_rplidar.sh)
- SLAM Toolbox

Odometry: wasd_teleop.py tarafından sağlanır (ayrı terminalde çalıştır).
Motor kontrol: wasd_teleop.py tarafından doğrudan serial ile yapılır.

Kullanım (Raspi'de):
  Terminal 1:
    ros2 launch my_robot_bringup slam_teleop.launch.py lidar_port:=/dev/ttyUSB0

  Terminal 2:
    python3 ~/REALNAV2.3-/src/my_robot_bringup/scripts/wasd_teleop.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction, ExecuteProcess, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_slam = FindPackageShare("slam_toolbox")

    # Arguments
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="RPLIDAR USB port"
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
        description="SLAM params yaml"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
    )

    lidar_port = LaunchConfiguration("lidar_port")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---------- Robot State Publisher (URDF → TF: base_link, laser_link etc.) ----------
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(
        Command(["xacro", " ", urdf_xacro_path]),
        value_type=str
    )
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---------- Joint State Publisher (Wheels) ----------
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---------- RPLIDAR A2M12 ----------
    rplidar_script = PathJoinSubstitution([pkg_bringup, "scripts", "start_rplidar.sh"])
    lidar_node = ExecuteProcess(
        cmd=['bash', rplidar_script, lidar_port],
        output='screen',
        sigterm_timeout='5',   # cleanup trap'e 5 saniye ver
        sigkill_timeout='3',   # ardından 3 saniye daha bekle
    )

    # ---------- SHUTDOWN GARANTİSİ: LiDAR motoru durdur ----------
    stop_lidar_script = PathJoinSubstitution([pkg_bringup, "scripts", "stop_lidar.sh"])
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['bash', stop_lidar_script, lidar_port],
                    output='screen',
                )
            ]
        )
    )

    # ---------- SLAM Toolbox (3s delay) ----------
    slam_toolbox_launch = TimerAction(
        period=3.0,
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
        declare_lidar_port,
        declare_slam_params,
        declare_use_sim_time,

        shutdown_handler,           # ← Launch kapanınca LiDAR'ı durdur (GARANTİ)
        robot_state_pub,            # URDF TF (base_link → laser_link etc.)
        joint_state_pub,            # Wheel TF
        lidar_node,                 # RPLIDAR A2M12
        slam_toolbox_launch,        # SLAM Toolbox
        # ros2_control YOK — odom TF wasd_teleop.py tarafından yayınlanır
    ])
