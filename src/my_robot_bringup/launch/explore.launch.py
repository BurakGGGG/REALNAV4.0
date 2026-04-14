import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config",
        "explore_params.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
    )
    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
    )
    ld.add_action(declare_use_sim_time)
    ld.add_action(explore_node)
    return ld
