#!/bin/bash
# ============================================================
#  REALNAV2 — Raspi bash aliases
#  ~/.bashrc'nin sonuna şunu ekle:
#    source /path/to/REALNAV4.0/robot_env.sh
# ============================================================

# Bu script, bulunduğu dizini otomatik kök kabul eder.
# İsterseniz dışarıdan override edebilirsiniz: export REALNAV_ROOT=/.../REALNAV4.0
export REALNAV_ROOT="${REALNAV_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$REALNAV_ROOT/fastdds_raspi.xml"

source /opt/ros/jazzy/setup.bash
source "$REALNAV_ROOT/install/setup.bash" 2>/dev/null || true

# --- Kısayollar ---
alias lidar='bash "$REALNAV_ROOT/src/my_robot_bringup/scripts/start_rplidar.sh" /dev/ttyUSB0'
alias slam='ros2 launch my_robot_bringup slam_teleop.launch.py'
alias nav2='ros2 launch my_robot_bringup real_robot_nav2.launch.py map:="$REALNAV_ROOT/maps/my_room_map.yaml"'
alias slamnav2='ros2 launch my_robot_bringup slam_nav2.launch.py'
alias teleop='python3 "$REALNAV_ROOT/src/my_robot_bringup/scripts/wasd_teleop.py"'
alias savemap='mkdir -p "$REALNAV_ROOT/maps" && ros2 run nav2_map_server map_saver_cli -f "$REALNAV_ROOT/maps/my_room_map"'
alias buildrobot='cd "$REALNAV_ROOT" && chmod +x src/my_robot_bringup/scripts/*.sh src/my_robot_bringup/scripts/*.py && colcon build --symlink-install && source install/setup.bash && cd -'
alias killrobot='pkill -9 -f "[r]plidar_composition"; pkill -9 -f slam_toolbox; pkill -9 -f wasd_teleop; pkill -9 -f nav2_; pkill -9 -f nav2_motor_bridge; echo "All robot processes killed."'

echo "🤖 REALNAV ortamı hazır. (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "   lidar    → LiDAR başlat (ayrı terminal)"
echo "   slam     → SLAM (haritalama, LiDAR dahil)"
echo "   slamnav2 → SLAM + Nav2 (LiDAR'ı önce ayrı başlat!)"
echo "   nav2     → Otonom Navigasyon (kayıtlı harita ile)"
echo "   teleop   → WASD klavye kontrol"
echo "   savemap  → Haritayı kaydet"
echo "   killrobot → Tüm process'leri kapat"
