"""
Lightweight SLAM Launch — ros2_control OLMADAN
================================================
- robot_state_publisher (URDF → TF)
- SLAM Toolbox

Odometry: wasd_teleop.py tarafından sağlanır (ayrı terminalde çalıştır).
Motor kontrol: wasd_teleop.py tarafından doğrudan serial ile yapılır.

Kullanım (Raspi'de):
  Terminal 1:
    lidar            # LiDAR'ı AYRI terminalde başlat (start_rplidar.sh)

  Terminal 2:
    slam             # slam_teleop.launch.py (bu dosya)

  Terminal 3:
    teleop           # wasd_teleop.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_slam = FindPackageShare("slam_toolbox")

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
        description="SLAM params yaml"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
    )

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

    # NOT: joint_state_publisher KALDIRILDI!
    # wasd_teleop.py zaten gerçek encoder verisinden odom TF yayınlıyor.
    # joint_state_publisher sıfır değer yayınlayarak çakışma yaratıyordu.

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
        declare_slam_params,
        declare_use_sim_time,

        robot_state_pub,            # URDF TF (base_link → laser_link etc.)
        # joint_state_pub KALDIRILDI — wasd_teleop.py odom TF'ini hallediyor
        slam_toolbox_launch,        # SLAM Toolbox
        # ros2_control YOK — odom TF wasd_teleop.py tarafından yayınlanır
    ])
