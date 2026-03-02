import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc    = FindPackageShare("my_robot_description")
    pkg_gz      = FindPackageShare("ros_gz_sim")
    pkg_nav2    = FindPackageShare("nav2_bringup")

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params_custom.yaml"]),
        description="Nav2 params yaml"
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_bringup, "rviz", "nav2.rviz"]),
        description="RViz config"
    )

    # --- Gazebo world ---
    world_path = PathJoinSubstitution([pkg_desc, "worlds", "example_walls_world.sdf"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gz, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [TextSubstitution(text="-r "), world_path]}.items(),
    )

    # --- Spawn robot ---
    model_sdf_path = PathJoinSubstitution([pkg_desc, "models", "my_robot", "model.sdf"])
    spawn_entity = Node(
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

    # --- Bridges ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- Robot state publisher ---
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(Command(["xacro", " ", urdf_xacro_path]), value_type=str)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # --- Lidar frame alias ---
    static_lidar_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0","0","0","0","0","0", "laser_link", "my_robot/laser_link/lidar"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # --- SLAM toolbox (mapping) ---
    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}],
    )

    # --- Nav2 navigation only (NO AMCL / NO map_server) ---
    # ---------- Navigation (Manual to exclude docking) ----------
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
    
    navigation.append(Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))
    navigation.append(Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]))

    navigation.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": nav_nodes},
        ],
    ))

    # --- Explorer ---
    explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("my_robot_explore"), "launch", "explore.launch.py"])
        )
    )

    # --- RViz ---
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        declare_params,
        declare_rviz,
        gz_sim,
        bridge,
        rsp,
        static_lidar_fix,
        TimerAction(period=4.0, actions=[spawn_entity]),
        TimerAction(period=7.0, actions=[slam]),
        TimerAction(period=8.0, actions=navigation),
        TimerAction(period=9.0, actions=[explorer]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
