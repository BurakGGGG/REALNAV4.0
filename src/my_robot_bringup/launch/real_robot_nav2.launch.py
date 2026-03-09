"""
Gerçek Robot Nav2 (Otonom Sürüş) Başlatma Dosyası
==================================================
autonomous_exploration.launch.py modelinde yeniden yapılandırıldı.

Zamanlama sırası:
  0s  → robot_state_publisher, joint_state_publisher, nav2_motor_bridge (hemen)
  0s  → LiDAR başlatma (start_rplidar.sh — retry ve port reset ile)
 20s  → Map Server + AMCL (LiDAR retry tamamlanmış olmalı)
 25s  → Nav2 node'ları (controller, planner, behavior, bt_navigator vb.)
 45s  → Lifecycle Managers (tüm node'lar ve TF hazır olduktan sonra)

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
    # 3. RPLIDAR A2M12 — Port reset + doğrudan Node launch
    #    ExecuteProcess(bash script) yerine Node kullanıyoruz çünkü
    #    Node doğrudan ROS2 DDS'e bağlanır, /scan topic kesin yayınlanır.
    # ================================================================
    # 3a. Önce portu resetle (eski kalıntıları temizle)
    port_reset = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f rplidar_composition 2>/dev/null; sleep 0.5; '
             'if [ -e /dev/ttyUSB0 ]; then '
             '  sudo -n chmod 666 /dev/ttyUSB0 2>/dev/null; '
             '  stty -F /dev/ttyUSB0 256000 raw -echo 2>/dev/null; '
             '  printf "\\xa5\\x25" > /dev/ttyUSB0 2>/dev/null; sleep 0.3; '
             '  printf "\\xa5\\x40" > /dev/ttyUSB0 2>/dev/null; sleep 0.5; '
             'fi; echo "[LiDAR] Port reset tamamlandı."'],
        output='screen',
    )

    # 3b. 3 saniye sonra rplidar Node'u başlat
    lidar_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rplidar_ros",
                executable="rplidar_composition",
                name="rplidar_node",
                parameters=[
                    {"serial_port": lidar_port},
                    {"serial_baudrate": 256000},
                    {"frame_id": "laser_link"},
                    {"inverted": False},
                    {"angle_compensate": True},
                    {"scan_mode": "Standard"},
                ],
                output="screen",
            )
        ]
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
        period=20.0,  # 20s: LiDAR retry (~15s) + Odom hazır olsun, sonra AMCL aç
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
        period=25.0,  # 25s: AMCL 20s'de başladı, 5s sonra navigation node'larını aç
        actions=nav2_nodes
    )

    # ================================================================
    # 6. Lifecycle Managers (ayrı ayrı — birinin hatası diğerini engellemez)
    #    Referans: articubot_one — localization + navigation ayrı lifecycle
    # ================================================================
    
    # 6a. Localization Lifecycle Manager (22s — map_server + amcl)
    localization_managed = ["map_server", "amcl"]
    lifecycle_localization = TimerAction(
        period=22.0,  # 22s: localization node'lar 20s'de başladı, 2s configure süresi
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

    # 6b. Navigation Lifecycle Manager (45s — AMCL TF hazır olduktan sonra)
    #     AMCL scan alıp map->odom TF yayınlaması ~30s sürebilir (LiDAR retry dahil)
    #     45s'de başlatınca global_costmap transform arayışında timeout olmaz
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
        period=35.0,  # 35s: Nav2 node'ları 25s'de başladı, 10s configure süresi
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
        port_reset,                # 0s — Port reset (eski kalıntıları temizle)
        lidar_node,                # 3s — RPLIDAR A2M12 (Node olarak)

        # Kademeli başlatma
        localization_launch,       # 20s — Map Server + AMCL
        lifecycle_localization,    # 22s — Localization aktifleştir (harita gelir)
        nav2_navigation,           # 25s — Controller, Planner, Behavior vb.
        lifecycle_navigation,      # 35s — Navigation aktifleştir (TF hazır)
    ])
