#!/bin/bash
# ============================================================
#  REALNAV2 — PC bash aliases
#  ~/.bashrc'nin sonuna şunu ekle:
#    source /path/to/REALNAV4.0/pc_env.sh
# ============================================================

# Bu script, bulunduğu dizini otomatik kök kabul eder.
# İsterseniz dışarıdan override edebilirsiniz: export REALNAV_ROOT=/.../REALNAV4.0
export REALNAV_ROOT="${REALNAV_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$REALNAV_ROOT/fastdds_pc.xml"
# Harita almak için (PC -> Raspi). Override: export REALNAV_PI_HOST=raspi@<ip>
export REALNAV_PI_HOST="${REALNAV_PI_HOST:-raspi@192.168.68.111}"

source /opt/ros/jazzy/setup.bash
source "$REALNAV_ROOT/install/setup.bash" 2>/dev/null || true

# --- Kısayollar ---
alias rvizslam='ros2 launch my_robot_bringup pc_rviz_slam.launch.py'
alias rviznav2='ros2 run rviz2 rviz2 -d "$REALNAV_ROOT/src/my_robot_bringup/rviz/exploration.rviz"'
alias getmap='mkdir -p "$REALNAV_ROOT/maps" && scp "$REALNAV_PI_HOST:~/REALNAV4.0/maps/my_room_map.*" "$REALNAV_ROOT/maps/"'
alias buildpc='cd "$REALNAV_ROOT" && colcon build --symlink-install && source install/setup.bash && cd -'
alias rosnodes='ros2 node list'
alias rostopics='ros2 topic list'

echo "🖥️  REALNAV PC ortamı hazır. (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "   rvizslam → RViz SLAM görselleştirme"
echo "   getmap   → Haritayı Raspi'den indir"
