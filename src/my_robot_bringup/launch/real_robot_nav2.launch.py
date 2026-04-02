"""
Gerçek Robot Nav2 (Otonom Sürüş) Başlatma Dosyası
==================================================
Bu dosya SADECE Nav2 + Map Server + AMCL + TF zincirini açar.
LiDAR daha ÖNCE ayrı bir terminalde `lidar` komutu ile başlatılmalıdır.

Zamanlama sırası:
  0s  → robot_state_publisher, joint_state_publisher, nav2_motor_bridge (hemen)
  20s → Map Server + AMCL (LiDAR ve odom hazır varsayılır)
  25s → Nav2 node'ları (controller, planner, behavior, bt_navigator vb.)
  35s → Lifecycle Managers (tüm node'lar ve TF hazır olduktan sonra)

Önerilen kullanım:
  Terminal 1 (Raspi):
    lidar

  Terminal 2 (Raspi):
    slam          # Haritalama yap

  Terminal 3 (Raspi):
    nav2          # Kayıtlı haritayla Nav2 (bu dosya)
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

    # ========== Arguments ==========
    declare_map = DeclareLaunchArgument(
        "map",
        description="Tam yol ile harita yaml dosyasi (/abs/path/.../map.yaml)"
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_custom.yaml"]),
        )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
    )
    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyAMA0",
        description="STM32 serial port yolu"
    )

    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")

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
    # NOT: joint_state_publisher KALDIRILDI!
    # nav2_motor_bridge zaten gerçek encoder verisinden /joint_states yayınlıyor.
    # joint_state_publisher sıfır/sabit değer yayınlayarak çakışma yaratıyordu
    # ve robot_state_publisher'a çelişkili TF verileri gidiyordu.
    # Bu, AMCL'nin map→odom TF'inin kaymasına neden oluyordu.

    # ================================================================
    # 2. Serial Bridge - cmd_vel -> STM32 & Odom/TF (0 saniye)
    # ================================================================
    nav2_bridge_node = Node(
        package="my_robot_bringup",
        executable="nav2_motor_bridge.py",
        output="screen",
        parameters=[
            {"serial_port": serial_port},
            {"baud_rate": 115200},
            {"pwm_multiplier": 318},
            {"wheel_radius": 0.051},
            {"wheel_separation": 0.43},
            {"ticks_per_rev": 4000},
        ]
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
        period=8.0,  # 20.0 → 8.0
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
        period=16.0,  # 25.0 → 16.0
        actions=nav2_nodes
    )

    # ================================================================
    # 6. Lifecycle Managers (ayrı ayrı — birinin hatası diğerini engellemez)
    #    Referans: articubot_one — localization + navigation ayrı lifecycle
    # ================================================================
    
    # 6a. Localization Lifecycle Manager (22s — map_server + amcl)
    localization_managed = ["map_server", "amcl"]
    lifecycle_localization = TimerAction(
        period=12.0,  # 22.0 → 12.0
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
        period=22.0,  # 35.0 → 22.0
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
        declare_nav2_params,
        declare_use_sim_time,
        declare_serial_port,

        # 0s — Hemen başlayanlar
        robot_state_pub,           # URDF → TF ağacı
        # joint_state_pub KALDIRILDI — nav2_bridge_node zaten /joint_states yayınlıyor
        nav2_bridge_node,          # cmd_vel → STM32, odom → TF → /joint_states

        # Kademeli başlatma
        localization_launch,       # 20s — Map Server + AMCL
        lifecycle_localization,    # 22s — Localization aktifleştir (harita gelir)
        nav2_navigation,           # 25s — Controller, Planner, Behavior vb.
        lifecycle_navigation,      # 35s — Navigation aktifleştir (TF hazır)
    ])
