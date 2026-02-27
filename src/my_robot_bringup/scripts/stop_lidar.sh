#!/bin/bash
# stop_lidar.sh — RPLidar motoru durdurma (standalone)
# Launch OnShutdown handler tarafından çağrılır.
# Her şey ölmüş olsa bile serial üzerinden motoru durdurur.

PORT="${1:-/dev/ttyUSB0}"

echo "[LiDAR-STOP] Motor durduruluyor..."

# 1. Tüm rplidar process'lerini öldür
pkill -9 -f rplidar_composition 2>/dev/null
sleep 1

# 2. Port serbest kaldıktan sonra STOP komutu gönder
if [ -e "$PORT" ]; then
    stty -F "$PORT" 256000 raw -echo 2>/dev/null
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    sleep 0.3
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    echo "[LiDAR-STOP] Serial STOP gönderildi."
else
    echo "[LiDAR-STOP] Port $PORT bulunamadı."
fi

echo "[LiDAR-STOP] Motor durduruldu."
