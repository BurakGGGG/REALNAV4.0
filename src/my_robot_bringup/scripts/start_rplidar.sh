#!/bin/bash
# RPLidar A2M12 başlatma script'i
# Retry ile başlatır. Cleanup'ta process'i öldürüp port serbest kaldıktan sonra
# serial STOP komutu gönderir.

PORT="${1:-/dev/ttyUSB0}"
MAX_ATTEMPTS=5
RPID=""
CLEANUP_RUN=0

cleanup() {
    [ "$CLEANUP_RUN" -eq 1 ] && return
    CLEANUP_RUN=1

    echo "[LiDAR] Kapatılıyor..."
    
    # 1. Process'i öldür — portu serbest bıraksın
    if [ -n "$RPID" ]; then
        kill $RPID 2>/dev/null
        # Process'in ölmesini bekle (max 3s)
        for x in $(seq 1 30); do
            kill -0 $RPID 2>/dev/null || break
            sleep 0.1
        done
        kill -9 $RPID 2>/dev/null
    fi
    pkill -9 -f rplidar_composition 2>/dev/null
    sleep 1
    
    # 2. Port artık serbest — serial STOP gönder
    if [ -e "$PORT" ]; then
        stty -F "$PORT" 256000 raw -echo 2>/dev/null
        printf '\xa5\x25' > "$PORT" 2>/dev/null
        sleep 0.2
        printf '\xa5\x25' > "$PORT" 2>/dev/null
    fi
    
    echo "[LiDAR] Durduruldu."
}

trap cleanup INT TERM EXIT

echo "[LiDAR] Port bekleniyor: $PORT"
for w in $(seq 1 15); do [ -e "$PORT" ] && break; sleep 1; done

if [ ! -e "$PORT" ]; then
    echo "[LiDAR] HATA: $PORT bulunamadı!"
    exit 1
fi

for i in $(seq 1 $MAX_ATTEMPTS); do
    echo "[LiDAR] ===== Deneme $i / $MAX_ATTEMPTS ====="

    pkill -9 -f rplidar_composition 2>/dev/null
    sleep 1

    ros2 run rplidar_ros rplidar_composition --ros-args \
        -p serial_port:="$PORT" \
        -p serial_baudrate:=256000 \
        -p frame_id:=laser_link \
        -p angle_compensate:=true \
        -p scan_mode:=Standard &
    RPID=$!

    echo "[LiDAR] 10s bekleniyor (PID=$RPID)..."
    sleep 10

    if ! kill -0 $RPID 2>/dev/null; then
        echo "[LiDAR] Deneme $i: Process erken kapandı."
        continue
    fi

    PUB_COUNT=$(ros2 topic info /scan 2>/dev/null | grep -oP 'Publisher count: \K\d+' || echo "0")
    if [ "$PUB_COUNT" -gt 0 ] 2>/dev/null; then
        echo "[LiDAR] BAŞARILI! (PID=$RPID)"
        wait $RPID
        exit 0
    else
        echo "[LiDAR] Deneme $i başarısız."
        kill -INT $RPID 2>/dev/null; sleep 1
    fi
done

echo "[LiDAR] Tüm denemeler başarısız!"
exit 1
