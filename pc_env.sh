#!/bin/bash
# ============================================================
#  REALNAV2 — PC bash aliases
#  ~/.bashrc'nin sonuna şunu ekle:
#    source ~/Desktop/REALNAV3-3/pc_env.sh
# ============================================================

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/Desktop/REALNAV3-3/fastdds_pc.xml

source /opt/ros/jazzy/setup.bash
source ~/Desktop/REALNAV3-3/install/setup.bash 2>/dev/null || true

# --- Kısayollar ---
alias rvizslam='ros2 launch my_robot_bringup pc_rviz_slam.launch.py'
alias rviznav2='ros2 run rviz2 rviz2 -d ~/Desktop/REALNAV3-3/src/my_robot_bringup/rviz/exploration.rviz'
alias getmap='scp raspi@192.168.68.111:~/REALNAV3-3/maps/my_room_map.* ~/Desktop/REALNAV3-3/maps/'
alias buildpc='cd ~/Desktop/REALNAV3-3 && colcon build --symlink-install && source install/setup.bash && cd -'
alias rosnodes='ros2 node list'
alias rostopics='ros2 topic list'

echo "🖥️  REALNAV2 PC ortamı hazır. (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "   rvizslam → RViz SLAM görselleştirme"
echo "   getmap   → Haritayı Raspi'den indir"
