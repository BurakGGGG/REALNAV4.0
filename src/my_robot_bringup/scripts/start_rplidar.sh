#!/bin/bash
# RPLidar A2M12 başlatma script'i — GÜVENİLİR VERSİYON v3
#
# Özellikler:
# - Port hazır değilse bekler (USB enumeration gecikmesi)
# - Port "busy" ise eski rplidar process'ini öldürür ve tekrar dener
# - 3 kez retry yapar, her retry arasında portu reset eder
# - SIGINT/SIGTERM gelince retry yapmadan temiz çıkar
# - Child process'leri (rplidar_composition) process group ile yönetir
#
# RPLidar Protokolü:
#   0xA5 0x25                       = STOP_SCAN (veri akışını durdurur, MOTOR DURMAZ!)
#   0xA5 0xF0 0x02 0x00 0x00 0x57  = SET_MOTOR_PWM(0) (MOTORU FİZİKSEL OLARAK DURDURUR)
#   0xA5 0x40                       = RESET (cihazı yeniden başlatır)

PORT="${1:-/dev/ttyUSB0}"
MAX_RETRIES=3
RETRY_DELAY=3
RPID=""
CLEANUP_RUN=0
SHUTTING_DOWN=0  # Signal geldi mi?

cleanup() {
    [ "$CLEANUP_RUN" -eq 1 ] && return
    CLEANUP_RUN=1
    SHUTTING_DOWN=1

    echo "[LiDAR] Kapatılıyor..."
    
    # 1. rplidar child process'ini öldür
    if [ -n "$RPID" ] && kill -0 $RPID 2>/dev/null; then
        kill $RPID 2>/dev/null
        for x in $(seq 1 30); do
            kill -0 $RPID 2>/dev/null || break
            sleep 0.1
        done
        kill -9 $RPID 2>/dev/null
    fi
    
    # 2. Yetim sllidar_node process'lerini de temizle ([s]llidar trick: kendi kendini öldürmesin)
    pkill -9 -f "[s]llidar_node" 2>/dev/null
    sleep 0.5
    
    echo "[LiDAR] Node/Motor durduruldu."
}

reset_port() {
    # Eski sllidar process'lerini öldür ([s]llidar trick: kendi kendini öldürmesin)
    pkill -9 -f "[s]llidar_node" 2>/dev/null
    sleep 1
}

trap cleanup INT TERM EXIT

# ===== Port Bekleme (max 20 saniye) =====
echo "[LiDAR] Port bekleniyor: $PORT"
for w in $(seq 1 20); do
    [ -e "$PORT" ] && break
    sleep 1
done

if [ ! -e "$PORT" ]; then
    echo "[LiDAR] HATA: $PORT bulunamadı! USB bağlantısını kontrol edin."
    exit 1
fi

# ===== Port İzin Kontrolü (şifresiz, launch bloklamaz) =====
if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "[LiDAR] UYARI: $PORT için okuma/yazma izni yok. Düzeltiliyor..."
    # 1. Önce sudo şifresiz dene (sudoers kuralı varsa çalışır)
    sudo -n chmod 666 "$PORT" 2>/dev/null
    if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
        # 2. Hâlâ izin yoksa kullanıcıya kalıcı çözüm göster ve çık
        echo "[LiDAR] HATA: İzin düzeltme başarısız!"
        echo "[LiDAR] Kalıcı çözüm için şu komutları çalıştır:"
        echo "  sudo usermod -a -G dialout $USER"
        echo "  echo 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/99-rplidar.rules"
        echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
        echo "  (Ardından logout/login yap)"
        exit 1
    fi
fi

# ===== RPLIDAR Başlatma (retry ile) =====
for attempt in $(seq 1 $MAX_RETRIES); do
    # Signal geldiyse retry yapma
    [ "$SHUTTING_DOWN" -eq 1 ] && exit 0

    echo "[LiDAR] RPLIDAR başlatılıyor (deneme $attempt/$MAX_RETRIES)..."

    # Önceki kalıntıları temizle
    reset_port

    # Signal kontrolü (reset_port sırasında gelmiş olabilir)
    [ "$SHUTTING_DOWN" -eq 1 ] && exit 0

    ros2 run sllidar_ros2 sllidar_node --ros-args \
        -p serial_port:="$PORT" \
        -p serial_baudrate:=256000 \
        -p frame_id:=laser_link \
        -p angle_compensate:=true &
    RPID=$!

    # 5 saniye bekle ve process'in hâlâ çalıştığını kontrol et
    for i in $(seq 1 10); do
        sleep 0.5
        [ "$SHUTTING_DOWN" -eq 1 ] && exit 0
        # Process öldüyse erken çık
        kill -0 $RPID 2>/dev/null || break
    done

    if kill -0 $RPID 2>/dev/null; then
        echo "[LiDAR] Çalışıyor (PID=$RPID). /scan yayınını kontrol edebilirsin."
        # Başarılı — process'i bekle
        wait $RPID
        EXIT_CODE=$?
        
        # Signal kaynaklı ölüm mü? (130=SIGINT, 143=SIGTERM)
        # Signal → temiz çıkış, retry yapma
        if [ $EXIT_CODE -eq 130 ] || [ $EXIT_CODE -eq 143 ] || [ "$SHUTTING_DOWN" -eq 1 ]; then
            echo "[LiDAR] Signal ile durduruldu (exit=$EXIT_CODE)."
            exit 0
        fi
        
        # Başka bir nedenle öldüyse ve retry hakkımız varsa tekrar dene
        if [ $attempt -lt $MAX_RETRIES ] && [ $EXIT_CODE -ne 0 ]; then
            echo "[LiDAR] Process öldü (exit=$EXIT_CODE). Yeniden deneniyor..."
            RPID=""
            sleep $RETRY_DELAY
            continue
        fi
        exit $EXIT_CODE
    else
        echo "[LiDAR] Başlatma başarısız (deneme $attempt/$MAX_RETRIES)."
        RPID=""
        
        if [ $attempt -lt $MAX_RETRIES ]; then
            echo "[LiDAR] ${RETRY_DELAY}s sonra tekrar denenecek..."
            sleep $RETRY_DELAY
        fi
    fi
done

echo "[LiDAR] HATA: ${MAX_RETRIES} denemede de başlatılamadı!"
echo "[LiDAR] Olası nedenler:"
echo "  1. USB kablosu gevşek veya arızalı"
echo "  2. Başka bir process portu kullanıyor (lsof $PORT)"
echo "  3. LiDAR donanım arızası"
exit 1
