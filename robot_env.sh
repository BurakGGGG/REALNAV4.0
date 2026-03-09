#!/bin/bash
# ============================================================
#  REALNAV2 — Raspi bash aliases
#  ~/.bashrc'nin sonuna şunu ekle:
#    source ~/REALNAV3-3/robot_env.sh
# ============================================================

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/REALNAV3-3/fastdds_raspi.xml

source /opt/ros/jazzy/setup.bash
source ~/REALNAV3-3/install/setup.bash 2>/dev/null || true

# --- Kısayollar ---
alias lidar='bash ~/REALNAV3-3/src/my_robot_bringup/scripts/start_rplidar.sh /dev/ttyUSB0'
alias slam='ros2 launch my_robot_bringup slam_teleop.launch.py lidar_port:=/dev/ttyUSB0'
alias nav2='ros2 launch my_robot_bringup real_robot_nav2.launch.py map:=/home/raspi/REALNAV3-3/maps/my_room_map.yaml lidar_port:=/dev/ttyUSB0'
alias slamnav2='ros2 launch my_robot_bringup slam_nav2.launch.py'
alias teleop='python3 ~/REALNAV3-3/src/my_robot_bringup/scripts/wasd_teleop.py'
alias savemap='mkdir -p ~/REALNAV3-3/maps && ros2 run nav2_map_server map_saver_cli -f ~/REALNAV3-3/maps/my_room_map'
alias buildrobot='cd ~/REALNAV3-3 && chmod +x src/my_robot_bringup/scripts/*.sh src/my_robot_bringup/scripts/*.py && colcon build --symlink-install && source install/setup.bash && cd -'
alias killrobot='sudo stty -F /dev/ttyUSB0 256000 raw -echo 2>/dev/null; sudo bash -c "printf \"\\xa5\\x25\" > /dev/ttyUSB0" 2>/dev/null; sudo bash -c "printf \"\\xa5\\xf0\\x02\\x00\\x00\\x57\" > /dev/ttyUSB0" 2>/dev/null; pkill -9 -f "[r]plidar_composition"; pkill -9 -f slam_toolbox; pkill -9 -f wasd_teleop; pkill -9 -f nav2_; pkill -9 -f nav2_motor_bridge; sleep 0.5; sudo bash -c "printf \"\\xa5\\xf0\\x02\\x00\\x00\\x57\" > /dev/ttyUSB0" 2>/dev/null; echo "All robot processes killed + LiDAR motor stopped."'

echo "🤖 REALNAV2 ortamı hazır. (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "   lidar    → LiDAR başlat (ayrı terminal)"
echo "   slam     → SLAM (haritalama, LiDAR dahil)"
echo "   slamnav2 → SLAM + Nav2 (LiDAR'ı önce ayrı başlat!)"
echo "   nav2     → Otonom Navigasyon (kayıtlı harita ile)"
echo "   teleop   → WASD klavye kontrol"
echo "   savemap  → Haritayı kaydet"
echo "   killrobot → Tüm process'leri kapat"
