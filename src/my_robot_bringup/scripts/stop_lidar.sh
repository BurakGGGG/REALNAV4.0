#!/bin/bash
# stop_lidar.sh — RPLidar motoru durdurma (güvenilir versiyon)
# Launch OnShutdown handler tarafından çağrılır.
# Her şey ölmüş olsa bile serial üzerinden motoru durdurur.

PORT="${1:-/dev/ttyUSB0}"

echo "[LiDAR-STOP] Motor durduruluyor..."

# 1. Tüm rplidar process'lerini düzgün öldür (SIGTERM önce, sonra SIGKILL)
pkill -f rplidar_composition 2>/dev/null
sleep 1
pkill -9 -f rplidar_composition 2>/dev/null
sleep 0.5

# 2. Port serbest kaldıktan sonra STOP komutu gönder (motoru fiziksel olarak durdur)
if [ -e "$PORT" ]; then
    stty -F "$PORT" 256000 raw -echo 2>/dev/null
    # STOP komutu: LiDAR motorunu durdurur
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    sleep 0.3
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    sleep 0.2
    # RESET komutu: LiDAR'ı fabrika durumuna getirir (bir sonraki açılış için temiz başlangıç)
    printf '\xa5\x40' > "$PORT" 2>/dev/null
    echo "[LiDAR-STOP] Serial STOP + RESET gönderildi."
else
    echo "[LiDAR-STOP] Port $PORT bulunamadı (USB çekilmiş olabilir)."
fi

echo "[LiDAR-STOP] Motor durduruldu."
