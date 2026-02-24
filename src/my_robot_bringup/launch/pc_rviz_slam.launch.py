"""
PC Tarafı RViz2 Launch Dosyası
Raspberry Pi'deki robot topic'lerini görselleştirmek için.

ROS_DOMAIN_ID Raspi ile aynı olmalı!
robot_state_publisher Raspi'de zaten çalışıyor, burada SADECE RViz2 başlatılır.

Kullanım (PC'de):
export ROS_DOMAIN_ID=0
ros2 launch my_robot_bringup pc_rviz_slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("my_robot_bringup")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # RViz2 - Raspi'deki topic'leri görselleştirir
    # robot_state_publisher Raspi'de çalışıyor, TF ve /robot_description
    # DDS üzerinden otomatik gelir (aynı ROS_DOMAIN_ID)
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
        rviz_node,
    ])
