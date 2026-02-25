"""
Gerçek Robot Otonom SLAM Launch Dosyası - ROS2 Control Mantığı
STM32 + RPLidar A2M12 + SLAM + Nav2 + Frontier Explorer

Bu launch dosyası:
1. real_robot_control_bringup.launch.py'yi include eder (ROS2 Control, TF, vb.)
2. RPLidar A2M12 node başlatır (/dev/ttyUSB0)
3. SLAM Toolbox ile harita oluşturur
4. Nav2 ile navigasyon sağlar
5. frontier_explorer ile otonom keşif yapar
NOT: RViz2 Raspberry Pi'de çalıştırılmaz, kendi bilgisayarında çalıştırılacak

Kullanım:
ros2 launch my_robot_bringup real_robot_control_slam.launch.py \
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
        description="Use simulation time"
    )
    
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value=TextSubstitution(text="true"),
        description="Automatically startup the nav2 stack"
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params.yaml"]),
        description="Nav2 params yaml"
    )
    
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
        description="SLAM params yaml"
    )
    
    declare_explore_params = DeclareLaunchArgument(
        "explore_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "explore_params.yaml"]),
        description="Exploration params yaml"
    )
    
    # Launch Configurations
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    lidar_port = LaunchConfiguration("lidar_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    explore_params_file = LaunchConfiguration("explore_params_file")
    
    # ---------- Include: real_robot_control_bringup.launch.py ----------
    # ROS2 Control, serial bridge, robot_state_publisher, controllers
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
    
    # ---------- LIDAR Node (RPLIDAR A2) ----------
    # ExecuteProcess + bash retry: Node action portu kilitliyor başarısız olunca
    lidar_node = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 3 && '
             'for i in 1 2 3 4 5; do '
             '  echo "[LiDAR] Attempt $i: starting rplidar..." && '
             '  ros2 run rplidar_ros rplidar_composition --ros-args '
             '    -p serial_port:=/dev/ttyUSB0 '
             '    -p serial_baudrate:=256000 '
             '    -p frame_id:=laser_link '
             '    -p angle_compensate:=true '
             '    -p scan_mode:=Standard '
             '  && break; '
             '  echo "[LiDAR] Attempt $i failed, waiting 2s..." && sleep 2; '
             'done'],
        output='screen',
    )
    
    # ---------- SLAM Toolbox ----------
    slam_toolbox_launch = TimerAction(
        period=2.0,  # 2 saniye bekle (TF tree hazır olsun)
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
    
    # ---------- Nav2 Navigation Stack ----------
    nav_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor"
    ]
    
    navigation = []
    
    navigation.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": nav_nodes},
        ],
    ))
    
    nav2_navigation = TimerAction(
        period=4.0,  # 4 saniye bekle (SLAM hazır olsun)
        actions=navigation
    )
    
    # ---------- Frontier Explorer ----------
    explore_node = TimerAction(
        period=6.0,  # 6 saniye bekle (Nav2 hazır olsun)
        actions=[
            Node(
                package="my_robot_explore",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
            )
        ]
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_lidar_port,
        declare_use_sim_time,
        declare_autostart,
        declare_nav2_params,
        declare_slam_params,
        declare_explore_params,
        
        real_robot_control_bringup,  # 0 saniye - ROS2 Control + Serial Bridge
        lidar_node,                  # 1 saniye sonra - RPLIDAR A2
        slam_toolbox_launch,         # 2 saniye sonra - SLAM
        nav2_navigation,             # 4 saniye sonra - Nav2
        explore_node,                # 6 saniye sonra - Frontier Explorer
        # RViz2 kaldırıldı - kendi bilgisayarında çalıştırılacak
    ])
