"""
Gerçek Robot Bringup Launch Dosyası - ROS2 Control Mantığı
STM32 Serial Bridge + ROS2 Control

Bu launch dosyası:
1. Serial bridge node başlatır (STM32 ile haberleşme)
2. ros2_control_node başlatır (Controller Manager)
3. robot_state_publisher başlatır (URDF -> TF)
4. joint_state_broadcaster başlatır (/joint_states)
5. my_robot_base_controller başlatır (Diff drive controller)

Kullanım:
ros2 launch my_robot_bringup real_robot_control_bringup.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_explore = FindPackageShare("my_robot_explore")
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="STM32 Serial port (USB-UART modülü)"
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value=TextSubstitution(text="115200"),
        description="Serial baud rate for STM32"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time"
    )
    
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # ---------- Serial Bridge Node ----------
    # STM32 ile haberleşme, /cmd_vel alır, /odom ve /joint_states yayınlar
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
    
    # ---------- Robot Description (URDF) ----------
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(
        Command(["xacro", " ", urdf_xacro_path]),
        value_type=str
    )
    robot_description = {"robot_description": robot_desc}
    
    # ---------- Controller Config ----------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_bringup"),
            "config",
            "my_robot_controllers.yaml",
        ]
    )
    
    # ---------- ROS2 Control Node ----------
    # Controller Manager'ı başlatır, hardware interface'i yükler, controller'ları yönetir
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/my_robot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    
    # ---------- Robot State Publisher ----------
    # URDF'den TF yayınlar (static transform'lar)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    
    # ---------- Joint State Broadcaster ----------
    # /joint_states topic'ini yayınlar
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # ---------- Base Controller Spawner ----------
    # Diff drive controller'ı başlatır
    # /cmd_vel alır, odometry hesaplar ve yayınlar
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_robot_base_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        remappings=[("/my_robot_base_controller/cmd_vel_unstamped", "/cmd_vel")],
    )
    
    # ---------- Event Handlers ----------
    # Controller'ı joint_state_broadcaster'dan sonra başlat
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_use_sim_time,
        
      #  serial_bridge,              # STM32 serial bridge (iptal: ros2_control kullaniyoruz)
        control_node,               # ROS2 Control node
        robot_state_pub_node,       # Robot state publisher
        joint_state_broadcaster_spawner,  # Joint state broadcaster
        delay_robot_controller_after_joint_state,  # Base controller (joint_state'den sonra)
    ])
