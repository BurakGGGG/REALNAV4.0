"""
SLAM + Nav2 Otonom Navigasyon Launch — Gerçek Robot
====================================================
SLAM Toolbox ile eşzamanlı haritalama + Nav2 ile navigasyon.

TF zinciri:
  map → odom       : SLAM Toolbox (AMCL YOK!)
  odom → base_footprint : nav2_motor_bridge.py (encoder)
  base_footprint → ...  : robot_state_publisher (URDF)

Kullanım (Raspi'de):
  Terminal 1:  lidar         ← LiDAR'ı ayrı başlat
  Terminal 2:  slamnav2      ← SLAM + Nav2

  PC'de:
  rviznav2
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

    # ========== Arguments ==========
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_custom.yaml"]),
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
    )

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # ================================================================
    # 0. URDF & TF  (0s — anında başlar)
    #    base_footprint → base_link → laser_link / wheels  (static TF)
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
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # ================================================================
    # 1. Serial Motor Bridge  (0s — anında başlar)
    #    odom → base_footprint TF yayınlar  (encoder'dan)
    #    cmd_vel → STM32 PWM gönderir
    # ================================================================
    nav2_bridge_node = Node(
        package="my_robot_bringup",
        executable="nav2_motor_bridge.py",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyAMA0"},
            {"baud_rate": 115200},
            {"pwm_multiplier": 318},
            {"use_sim_time": False},
        ]
    )

    # ================================================================
    # 2. LiDAR → AYRI TERMİNALDEN BAŞLATILIR!
    #    Terminal 1'de:  lidar
    #    (start_rplidar.sh retry mekanizması ile güvenilir çalışır)
    # ================================================================

    # ================================================================
    # 3. SLAM Toolbox  (5s — LiDAR ve odom hazır olduktan sonra)
    #    map → odom TF yayınlar  (AMCL YOK!)
    #    /map topic yayınlar
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
    # 4. Nav2 Navigation Node'ları  (15s — SLAM'ın map→odom TF'i yayınlaması için yeterli süre)
    #    NOT: AMCL ve map_server YOK! SLAM Toolbox her ikisini de halleder.
    # ================================================================
    nav2_nodes = []

    nav2_nodes.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            nav2_params_file,
            {"use_sim_time": False},
            # Eski SLAM oturumundan kalan /map mesajını (transient_local) dikkate alma
            {"global_costmap.global_costmap.static_layer.map_subscribe_transient_local": False},
            # TF zaman toleransını artır — SLAM bazen TF'i gecikmeli yayınlar
            {"global_costmap.global_costmap.transform_tolerance": 5.0},
        ]
    ))
    nav2_nodes.append(Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))
    nav2_nodes.append(Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": False}]
    ))

    nav2_navigation = TimerAction(
        period=15.0,
        actions=nav2_nodes
    )

    # ================================================================
    # 5. Navigation Lifecycle Manager  (25s — tüm node'lar kesinlikle hazır)
    #    bond_timeout=120 → node'lara aktivasyon için bol süre ver
    # ================================================================
    navigation_managed = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
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

        # 0s — Hemen başlayanlar
        robot_state_pub,           # URDF → static TF ağacı
        joint_state_pub,           # Joint states (tekerlek TF)
        nav2_bridge_node,          # cmd_vel ↔ STM32, odom → TF

        # Kademeli başlatma
        # LiDAR → AYRI TERMİNALDEN: lidar komutu
        slam_toolbox_launch,       #  5s — SLAM Toolbox  (map → odom TF + /map)
        nav2_navigation,           # 15s — Nav2 node'ları  (AMCL YOK!)
        lifecycle_navigation,      # 25s — Navigation aktifleştir
    ])
