#!/bin/bash
# ============================================================
#  REALNAV2 — Raspi bash aliases
#  ~/.bashrc'nin sonuna şunu ekle:
#    source ~/REALNAV2.3-/robot_env.sh
# ============================================================

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/REALNAV2.3-/fastdds_raspi.xml

source /opt/ros/jazzy/setup.bash
source ~/REALNAV2.3-/install/setup.bash 2>/dev/null || true

# --- Kısayollar ---
alias slam='ros2 launch my_robot_bringup slam_teleop.launch.py lidar_port:=/dev/ttyUSB0'
alias teleop='python3 ~/REALNAV2.3-/src/my_robot_bringup/scripts/wasd_teleop.py'
alias savemap='mkdir -p ~/REALNAV2.3-/maps && ros2 run nav2_map_server map_saver_cli -f ~/REALNAV2.3-/maps/my_room_map'
alias buildrobot='cd ~/REALNAV2.3- && colcon build --symlink-install && source install/setup.bash && cd -'
alias killrobot='stty -F /dev/ttyUSB0 256000 raw -echo 2>/dev/null; printf "\xa5\x25" > /dev/ttyUSB0 2>/dev/null; pkill -9 -f rplidar_composition; pkill -9 -f slam_toolbox; pkill -9 -f wasd_teleop; sleep 0.3; printf "\xa5\x25" > /dev/ttyUSB0 2>/dev/null; echo "All robot processes killed + LiDAR motor stopped."'

echo "🤖 REALNAV2 ortamı hazır. (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "   slam     → SLAM launch"
echo "   teleop   → WASD klavye kontrol"
echo "   savemap  → Haritayı kaydet"
echo "   killrobot → Tüm robot process'lerini kapat"
