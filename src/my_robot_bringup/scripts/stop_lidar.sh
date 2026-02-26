#!/bin/bash
# LiDAR Motor Durdurma Script'i
# Launch kapanınca veya manuel olarak çağrılır
# Kullanım: stop_lidar.sh [port]

PORT="${1:-/dev/ttyUSB0}"

echo "[LiDAR-STOP] Motor durduruluyor..."

# 1. Seri porttan STOP komutu (0xA5 0x25 = stop motor)
if [ -e "$PORT" ]; then
    stty -F "$PORT" 256000 raw -echo 2>/dev/null
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    printf '\xa5\x25' > "$PORT" 2>/dev/null
fi

# 2. ROS servisini dene (arka planda, takılmasın)
timeout 1 ros2 service call /stop_motor std_srvs/srv/Empty 2>/dev/null &

# 3. Tüm rplidar process'lerini öldür
pkill -9 -f rplidar_composition 2>/dev/null

# 4. Son garanti — tekrar seri porttan STOP
sleep 0.3
if [ -e "$PORT" ]; then
    printf '\xa5\x25' > "$PORT" 2>/dev/null
fi

echo "[LiDAR-STOP] Tamamlandı."
