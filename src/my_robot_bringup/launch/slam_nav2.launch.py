"""
SLAM + Nav2 Otonom Navigasyon Launch — Gerçek Robot
====================================================
SLAM Toolbox ile eşzamanlı haritalama + Nav2 ile navigasyon.

TF zinciri:
  map → odom       : SLAM Toolbox (AMCL YOK!)
  odom → base_footprint : nav2_motor_bridge.py (encoder)
  base_footprint → ...  : robot_state_publisher (URDF)

Kullanım (Raspi'de):
  Terminal 1:  lidar
  Terminal 2:  slamnav2

  PC'de:
  rviznav2
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_slam = FindPackageShare("slam_toolbox")

    # ========== Arguments ==========
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        # DÜZELTME: config/ klasöründen, SLAM'a özel params dosyası
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_slam.yaml"]),
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
    )

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # ================================================================
    # 0. URDF & TF  (0s)
    # ================================================================
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
            {"use_sim_time": False},
        ],
    )

    # ================================================================
    # 1. Serial Motor Bridge  (0s)
    # ================================================================
    nav2_bridge_node = Node(
        package="my_robot_bringup",
        executable="nav2_motor_bridge.py",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyAMA0"},
            {"baud_rate": 115200},
            {"pwm_multiplier": 318},
            {"wheel_radius": 0.051},
            {"wheel_separation": 0.43},
            {"ticks_per_rev": 4000},
            {"use_sim_time": False},
        ]
    )

    # ================================================================
    # 3. SLAM Toolbox  (5s)
    # ================================================================
    slam_toolbox_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": "false",
                }.items()
            )
        ]
    )

    # ================================================================
    # 4. Nav2 Node'ları  (15s)
    # nav2_params_slam.yaml zaten static_layer içermiyor,
    # ek override'a gerek yok
    # ================================================================
    nav2_nodes = []

    for pkg, exe, name in [
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_smoother", "smoother_server", "smoother_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
        ("nav2_collision_monitor", "collision_monitor", "collision_monitor"),
    ]:
        nav2_nodes.append(Node(
            package=pkg, executable=exe, name=name,
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": False}]
        ))

    nav2_navigation = TimerAction(period=15.0, actions=nav2_nodes)

    # ================================================================
    # 5. Navigation Lifecycle Manager  (25s)
    # ================================================================
    navigation_managed = [
        "controller_server", "smoother_server", "planner_server",
        "behavior_server", "bt_navigator", "waypoint_follower",
        "velocity_smoother", "collision_monitor",
    ]
    lifecycle_navigation = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": True},
                    {"bond_timeout": 120.0},
                    {"node_names": navigation_managed},
                ],
            )
        ]
    )

    return LaunchDescription([
        declare_nav2_params,
        declare_slam_params,
        robot_state_pub,
        nav2_bridge_node,
        slam_toolbox_launch,
        nav2_navigation,
        lifecycle_navigation,
    ])
