#!/bin/bash
# RPLidar A2M12 başlatma script'i (DURDURMA GARANTİLİ - STRICT TIMEOUT + SERIAL)
# ===================================================

PORT="${1:-/dev/ttyUSB0}"
MAX_ATTEMPTS=5
RPID=""
CLEANUP_RUN=0

cleanup() {
    if [ "$CLEANUP_RUN" -eq 1 ]; then return; fi
    CLEANUP_RUN=1

    echo ""
    echo "[LiDAR] Kapatılıyor..."
    
    # 1. Hızlı Serial Çözüm (İlk bunu dene çünkü servis ölmüş olabilir)
    if [ -e "$PORT" ]; then
        echo "[LiDAR] Doğrudan seri porttan STOP gönderiliyor..."
        stty -F "$PORT" 256000 raw -echo 2>/dev/null
        printf '\xa5\x25' > "$PORT" 2>/dev/null
        sleep 0.2
    fi

    # 2. ROS 2 servisini TIMEOUT ile çağır (takılmayı önle)
    echo "[LiDAR] /stop_motor servisi deneniyor (max 2s)..."
    timeout 2 ros2 service call /stop_motor std_srvs/srv/Empty 2>/dev/null
    
    # 3. Process'i nazikçe durdur
    if [ ! -z "$RPID" ]; then
        kill -INT $RPID 2>/dev/null
    fi
    sleep 1
    
    # 4. Hala yaşıyorsa zorla öldür
    pkill -9 -f rplidar_composition 2>/dev/null
    
    # 5. Son garanti Serial Çözüm (Emin olmak için)
    if [ -e "$PORT" ]; then
        printf '\xa5\x25' > "$PORT" 2>/dev/null
    fi
    
    echo "[LiDAR] Tamamen durduruldu."
    exit 0
}

trap cleanup INT TERM EXIT

echo "[LiDAR] Port bekleniyor: $PORT"
for w in $(seq 1 15); do
    [ -e "$PORT" ] && break
    sleep 1
done

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
        echo "[LiDAR] BAŞARILI! /scan yayını var. (PID=$RPID)"
        # Process bitene kadar bekle
        wait $RPID
        exit 0
    else
        echo "[LiDAR] Deneme $i BAŞARISIZ (Yayın yok). Öldürülüyor..."
        kill -INT $RPID 2>/dev/null
        sleep 1
    fi
done

echo "[LiDAR] Tüm denemeler başarısız oldu!"
cleanup
exit 1
