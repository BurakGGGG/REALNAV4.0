import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc    = FindPackageShare("my_robot_description")
    pkg_gz      = FindPackageShare("ros_gz_sim")
    pkg_nav2    = FindPackageShare("nav2_bringup")

    # ---------- Args ----------
    default_map = os.path.join(os.path.expanduser("~"), "maps", "my_map.yaml")

    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value=TextSubstitution(text="slam"),  # slam | localize
        description="slam: slam_toolbox ile haritalama, localize: map_server+amcl ile localization"
    )

    declare_explore = DeclareLaunchArgument(
        "explore",
        default_value=TextSubstitution(text="false"),
        description="true ise frontier_explorer otonom keşif yapar"
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=TextSubstitution(text=default_map),
        description="Saved map yaml file (localize modunda kullanılır)"
    )

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

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="true"),
        description="Use simulation time"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---------- Gazebo world ----------
    world_path = PathJoinSubstitution([pkg_desc, "worlds", "example_walls_world.sdf"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gz, "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [TextSubstitution(text="-r "), world_path]}.items(),
    )

    # ---------- Spawn robot ----------
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

    # ---------- Bridges (tf bridge YOK) ----------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---------- Robot State Publisher (URDF -> TF) ----------
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

    # ---------- Odom -> base_footprint TF ----------
    odom_tf = Node(
        package="my_robot_explore",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---------- Static TF for lidar frame ----------
    static_lidar_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "laser_link", "--child-frame-id", "my_robot/laser_link/lidar"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---------- SLAM toolbox (mode==slam) ----------
    # ---------- SLAM toolbox (mode==slam) ----------
    pkg_slam = FindPackageShare("slam_toolbox")
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'slam'"])),
    )

    # ---------- Localization (mode==localize) ----------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav2, "launch", "localization_launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    localization = TimerAction(
        period=8.0,
        actions=[localization],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'localize'"])),
    )

    # ---------- Navigation (her iki modda da lazım) ----------
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
    
    navigation.append(Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))
    navigation.append(Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]))

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

    # ---------- Explorer (explore==true) ----------
    explorer = Node(
        package="my_robot_explore",
        executable="frontier_explorer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("explore")),
    )

    # ---------- RViz ----------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


    return LaunchDescription([
        declare_mode,
        declare_explore,
        declare_map,
        declare_params,
        declare_rviz,
        declare_use_sim_time,

        gz_sim,
        bridge,
        rsp,
        odom_tf,
        static_lidar_fix,
        jsp,

        TimerAction(period=4.0, actions=[spawn_entity]),
        TimerAction(period=6.0, actions=[slam]),          # slam modunda çalışır
        TimerAction(period=7.0, actions=navigation),
        localization,                                      # localize modunda çalışır
        TimerAction(period=9.0, actions=[explorer]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
