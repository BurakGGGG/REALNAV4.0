"""
Gercek Robot Serial Bringup Launch Dosyasi.

Bu launch dosyasi:
1. stm32_serial_bridge node'unu baslatir
2. robot_state_publisher node'unu baslatir
3. odom_tf_broadcaster node'unu baslatir
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = FindPackageShare("my_robot_description")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="STM32 serial port",
    )
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value=TextSubstitution(text="115200"),
        description="STM32 serial baud rate",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time",
    )

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_description = ParameterValue(Command(["xacro", " ", urdf_xacro_path]), value_type=str)

    serial_bridge = Node(
        package="my_robot_explore",
        executable="stm32_serial_bridge",
        name="stm32_serial_bridge",
        output="screen",
        parameters=[
            {"serial_port": serial_port},
            {"baud_rate": baud_rate},
            {"use_sim_time": use_sim_time},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ]
    )

    odom_tf_broadcaster = Node(
        package="my_robot_explore",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odom_topic": "/odom"},
            {"odom_frame": "odom"},
            {"base_frame": "base_footprint"},
        ],
    )

    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_use_sim_time,
        serial_bridge,
        robot_state_publisher,
        odom_tf_broadcaster,
    ])
