"""
Otonom SLAM Keşif Launch Dosyası
ros_gz_sim (Gazebo Gz) ile

Bu launch dosyası:
1. SLAM Toolbox ile harita oluşturur
2. Nav2 ile navigasyon sağlar
3. frontier_explorer ile otonom keşif yapar

Kullanım:
ros2 launch my_robot_bringup autonomous_exploration.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_gz = FindPackageShare("ros_gz_sim")
    pkg_nav2 = FindPackageShare("nav2_bringup")
    pkg_slam = FindPackageShare("slam_toolbox")
    
    # Konfigürasyon dosyası yolları
    nav2_config_path = LaunchConfiguration("nav2_params_file")
    slam_config_path = PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"])
    explore_config_path = PathJoinSubstitution([pkg_bringup, "config", "explore_params.yaml"])
    
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_custom.yaml"]),
        description="Nav2 params yaml"
    )
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="true"),
        description="Use simulation time"
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value=TextSubstitution(text="true"),
        description="Automatically startup the nav2 stack"
    )
    
    # 1. Gazebo World (0 saniye - anında başlar)
    world_path = PathJoinSubstitution([pkg_desc, "worlds", "example_walls_world.sdf"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gz, "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [TextSubstitution(text="-r "), world_path]}.items(),
    )
    
    # 2. Spawn Entity - Robot'u Gazebo'ya spawn et (4 saniye sonra)
    model_sdf_path = PathJoinSubstitution([pkg_desc, "models", "my_robot", "model.sdf"])
    spawn_entity = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "example_walls_world",
                    "-name", "my_robot",
                    "-file", model_sdf_path,
                    "-x", "0", "-y", "0", "-z", "1.0"
                ],
                output="screen",
            )
        ]
    )
    
    # 3. Bridge (GZ <-> ROS) - 0 saniye
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # 4. Robot State Publisher (URDF -> TF) - 0 saniye
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(Command(["xacro", " ", urdf_xacro_path]), value_type=str)
    
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_desc,
        }],
    )
    
    # 5. Odom -> base_footprint TF - 0 saniye
    odom_tf = Node(
        package="my_robot_explore",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # 6. Static TF for lidar frame - 0 saniye
    static_lidar_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "laser_link", "--child-frame-id", "my_robot/laser_link/lidar"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    # 7. Joint State Publisher - 0 saniye
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    # 8. SLAM Toolbox - Harita oluşturma (6 saniye sonra)
    slam_toolbox_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])
                ),
                launch_arguments={
                    "slam_params_file": slam_config_path,
                    "use_sim_time": use_sim_time,
                }.items()
            )
        ]
    )
    
    # 9. Nav2 Navigation Stack (7 saniye sonra) - Manual to exclude docking
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
    
    navigation.append(Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=[nav2_config_path, {"use_sim_time": use_sim_time}]))

    navigation.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": nav_nodes},
        ],
    ))

    nav2_navigation = TimerAction(
        period=7.0,
        actions=navigation
    )
    
    # 10. Frontier Explorer - Otonom Keşif (9 saniye sonra - SLAM'ın harita oluşturması için yeterli zaman)
    explore_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="my_robot_explore",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[explore_config_path, {"use_sim_time": use_sim_time}],
            )
        ]
    )
    
    # 11. RViz2 - Görselleştirme (11 saniye sonra) - Otonom keşif için optimize edilmiş
    rviz_config = PathJoinSubstitution([pkg_bringup, "rviz", "exploration.rviz"])
    rviz_node = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen"
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_nav2_params,
        
        gz_sim,                    # 0 saniye - anında başlar
        bridge,                    # 0 saniye
        rsp,                       # 0 saniye
        odom_tf,                   # 0 saniye
        static_lidar_fix,          # 0 saniye
        jsp,                       # 0 saniye
        spawn_entity,              # 4 saniye sonra
        slam_toolbox_launch,       # 6 saniye sonra
        nav2_navigation,           # 7 saniye sonra
        explore_node,              # 9 saniye sonra
        rviz_node,                 # 11 saniye sonra
    ])
