#!/bin/bash
# stop_lidar.sh — RPLidar A2M12 motoru durdurma (düzeltilmiş versiyon)
#
# RPLidar Protokolü:
#   0xA5 0x25        = STOP SCAN  (sadece veri akışını durdurur, MOTOR DURMAZ!)
#   0xA5 0xF0 0x02 0x00 0x00 0x57 = SET_MOTOR_PWM(0) (MOTORU DURDURUR)
#   0xA5 0x40        = RESET (cihazı yeniden başlatır)

PORT="${1:-/dev/ttyUSB0}"

echo "[LiDAR-STOP] Motor durduruluyor..."

# 1. sllidar_node process'lerini öldür
pkill -f "[s]llidar_node" 2>/dev/null
sleep 1
pkill -9 -f "[s]llidar_node" 2>/dev/null
sleep 0.5

# 2. Serial port üzerinden motor durdurma komutları gönder
if [ -e "$PORT" ]; then
    # Port ayarla
    stty -F "$PORT" 256000 raw -echo 2>/dev/null

    # STOP SCAN komutu (veri akışını durdur)
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    sleep 0.1

    # SET_MOTOR_PWM = 0  →  MOTORU FİZİKSEL OLARAK DURDUR
    # Paket: [0xA5] [0xF0] [0x02] [0x00 0x00] [checksum=0x57]
    printf '\xa5\xf0\x02\x00\x00\x57' > "$PORT" 2>/dev/null
    sleep 0.3

    # Tekrar STOP + Motor PWM=0 (garanti olsun)
    printf '\xa5\x25' > "$PORT" 2>/dev/null
    sleep 0.1
    printf '\xa5\xf0\x02\x00\x00\x57' > "$PORT" 2>/dev/null
    sleep 0.2

    echo "[LiDAR-STOP] STOP_SCAN + SET_MOTOR_PWM(0) gönderildi."
else
    echo "[LiDAR-STOP] Port $PORT bulunamadı (USB çekilmiş olabilir)."
fi

echo "[LiDAR-STOP] Motor durduruldu."
