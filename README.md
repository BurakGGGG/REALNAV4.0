1. Raspberry Pi Üzerinde (Robot Tarafı)
Robotun ana sistemlerini ve LiDAR birimini başlatmak için iki ayrı terminal kullanın.

Terminal 1: Ana Launch (Robot Bringup)

Bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/REALNAV2.3-/fastdds_raspi.xml
source ~/REALNAV2.3-/install/setup.bash

ros2 launch my_robot_bringup real_robot_lidar_slam.launch.py \
    serial_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0
Terminal 2: LiDAR (Ayrı Çalıştırma)

Bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/REALNAV2.3-/fastdds_raspi.xml
source ~/REALNAV2.3-/install/setup.bash

ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=256000 \
    -p frame_id:=laser_link \
    -p angle_compensate:=true \
    -p scan_mode:=Standard
2. PC Üzerinde (Kontrol Merkezi)
Haritayı görselleştirmek ve verileri doğrulamak için PC tarafında şu komutları kullanın.

Terminal 1: RViz & SLAM Görselleştirme

Bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/Desktop/REALNAV2.3-/fastdds_pc.xml
source ~/Desktop/REALNAV2.3-/install/setup.bash

ros2 launch my_robot_bringup pc_rviz_slam.launch.py
Terminal 2: Veri Doğrulama
Aşağıdaki komutlarla sistemin haberleştiğini ve veri akışını kontrol edebilirsiniz:

Bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/Desktop/REALNAV2.3-/fastdds_pc.xml
source ~/Desktop/REALNAV2.3-/install/setup.bash

# Aktif başlıkları listele
ros2 topic list

# LiDAR veri hızını kontrol et
ros2 topic hz /scan
