"""
Gerçek Robot Nav2 (Otonom Sürüş) Başlatma Dosyası
==================================================
autonomous_exploration.launch.py modelinde yeniden yapılandırıldı.

Zamanlama sırası:
  0s  → robot_state_publisher, joint_state_publisher, nav2_motor_bridge (hemen)
  0s  → LiDAR başlatma (start_rplidar.sh — retry ve port reset ile)
 12s  → Map Server + AMCL (LiDAR retry tamamlanmış olmalı)
 15s  → Nav2 node'ları (controller, planner, behavior, bt_navigator vb.)
 18s  → Lifecycle Manager (tüm node'lar hazır olduktan sonra)

Kullanım:
ros2 launch my_robot_bringup real_robot_nav2.launch.py map:=/home/raspi/REALNAV2.3-/maps/my_room_map.yaml
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

    # ========== Arguments ==========
    declare_map = DeclareLaunchArgument(
        "map",
        description="Tam yol ile harita yaml dosyasi (/home/raspi/.../map.yaml)"
    )
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_custom.yaml"]),
        )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
    )

    map_file = LaunchConfiguration("map")
    lidar_port = LaunchConfiguration("lidar_port")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ================================================================
    # 1. URDF & TF (0 saniye - anında başlar)
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
            {"use_sim_time": use_sim_time},
        ],
    )
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ================================================================
    # 2. Serial Bridge - cmd_vel -> STM32 & Odom/TF (0 saniye)
    # ================================================================
    nav2_bridge_node = Node(
        package="my_robot_bringup",
        executable="nav2_motor_bridge.py",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyAMA0"},
            {"baud_rate": 115200},
            {"pwm_multiplier": 318}  # örn: 255 / 0.8 m/s
        ]
    )

    # ================================================================
    # 3. RPLIDAR A2M12 (0 saniye - script kendi içinde portu bekler)
    # ================================================================
    rplidar_script = PathJoinSubstitution([pkg_bringup, "scripts", "start_rplidar.sh"])
    lidar_node = ExecuteProcess(
        cmd=['bash', rplidar_script, lidar_port],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='3',
    )
    stop_lidar_script = PathJoinSubstitution([pkg_bringup, "scripts", "stop_lidar.sh"])
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(cmd=['bash', stop_lidar_script, lidar_port], output='screen')
            ]
        )
    )

    # ================================================================
    # 4. Map Server + AMCL (5 saniye sonra)
    #    LiDAR ve odom TF'in hazır olmasını bekler
    # ================================================================
    localization_nodes = []

    localization_nodes.append(Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params_file, {"yaml_filename": map_file}, {"use_sim_time": use_sim_time}]
    ))

    localization_nodes.append(Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))

    localization_launch = TimerAction(
        period=12.0,  # 12s: LiDAR retry + Odom hazır olsun, sonra AMCL aç
        actions=localization_nodes
    )

    # ================================================================
    # 5. Nav2 Navigation Node'ları (8 saniye sonra)
    #    AMCL'nin map -> odom TF yayınına başlamasını bekler
    # ================================================================
    nav2_nodes = []

    nav2_nodes.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    nav2_nodes.append(Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))

    nav2_navigation = TimerAction(
        period=15.0,  # 15s: AMCL map->odom TF hazır, navigation node'larını aç
        actions=nav2_nodes
    )

    # ================================================================
    # 6. Lifecycle Managers (ayrı ayrı — birinin hatası diğerini engellemez)
    #    Referans: articubot_one — localization + navigation ayrı lifecycle
    # ================================================================
    
    # 6a. Localization Lifecycle Manager (14s — map_server + amcl)
    localization_managed = ["map_server", "amcl"]
    lifecycle_localization = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"bond_timeout": 60.0},
                    {"node_names": localization_managed},
                ],
            )
        ]
    )

    # 6b. Navigation Lifecycle Manager (20s — nav2 node'ları)
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
        period=20.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"bond_timeout": 60.0},
                    {"node_names": navigation_managed},
                ],
            )
        ]
    )

    return LaunchDescription([
        declare_map,
        declare_lidar_port,
        declare_nav2_params,
        declare_use_sim_time,

        # 0s — Hemen başlayanlar
        shutdown_handler,
        robot_state_pub,           # URDF → TF ağacı
        joint_state_pub,           # Joint states
        nav2_bridge_node,          # cmd_vel → STM32, odom → TF
        lidar_node,                # RPLIDAR A2M12 (script kendi portu bekler)

        # Kademeli başlatma
        localization_launch,       # 12s — Map Server + AMCL
        lifecycle_localization,    # 14s — Localization aktifleştir (harita gelir)
        nav2_navigation,           # 15s — Controller, Planner, Behavior vb.
        lifecycle_navigation,      # 20s — Navigation aktifleştir
    ])
