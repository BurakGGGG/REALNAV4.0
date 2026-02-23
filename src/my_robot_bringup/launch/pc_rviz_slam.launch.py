"""
PC Tarafı RViz2 Launch Dosyası
Raspberry Pi'deki robot topic'lerini görselleştirmek için.

ROS_DOMAIN_ID Raspi ile aynı olmalı!

Kullanım (PC'de):
export ROS_DOMAIN_ID=0
ros2 launch my_robot_bringup pc_rviz_slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_bringup = FindPackageShare("my_robot_bringup")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # URDF
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(
        Command(["xacro", " ", urdf_xacro_path]),
        value_type=str
    )
    
    # Robot State Publisher - URDF'den TF yayınlar
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )
    
    # RViz2
    rviz_config = PathJoinSubstitution([pkg_bringup, "rviz", "slam_view.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_pub,
        rviz_node,
    ])
