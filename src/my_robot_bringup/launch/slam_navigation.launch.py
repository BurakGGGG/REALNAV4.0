import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("my_robot_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    declare_slam = DeclareLaunchArgument(
        "slam", default_value="True",
        description="True: SLAM (mapping). False: AMCL localization (saved map)."
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True"
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_pkg, "config", "nav2_params_custom.yaml")
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_pkg, "maps", "my_map.yaml"),
        description="Map yaml path (used when slam:=False)"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "gazebo_slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_name": "example_walls_world",
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam,
            "map": map_yaml,
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "True",
        }.items(),
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", "nav2.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        declare_slam,
        declare_use_sim_time,
        declare_params,
        declare_map,
        gazebo_launch,
        nav2_launch,
        rviz,
    ])
