# REALNAV2 — SLAM & Navigation Sistemi

Bu proje, Raspberry Pi 5 ve STM32 bazlı gerçek bir robot için ROS 2 Jazzy üzerinde SLAM ve Navigasyon çözümü sunar.

## 🛜 Ağ ve Ortam Ayarları
- **Raspi IP:** `192.168.68.111`
- **PC IP:** `192.168.68.118` (veya güncel PC IP'niz)
- **Domain ID:** `0`
- **DDS:** Unicast (FastDDS XML profilleri kullanılır)

### Tek Seferlik Kurulum
Hem PC hem Raspi'de terminallere girip `.bashrc` dosyalarının sonuna şu satırları eklediğinizden emin olun:

**PC (~/.bashrc):**
```bash
source ~/Desktop/REALNAV2.3-/pc_env.sh
```

**Raspi (~/.bashrc):**
```bash
source ~/REALNAV2.3-/robot_env.sh
```

---

## 🚀 Çalıştırma Sırası (Sıralama Önemlidir)

### 1. Donanım Hazırlığı
- STM32'nin gücünü kesin ve tekrar takın (Power Reset).

### 2. Raspi — Terminal 1: Teleop & Odom
Önce `wasd_teleop.py`'yi başlatın. Bu, tekerlek kontrolünü ve Odometry (odom → base_footprint) yayınını başlatır.
```bash
teleop
```

### 3. Raspi — Terminal 2: SLAM Launch
LiDAR'ı ve SLAM Toolbox'ı başlatır. LiDAR motoru `SUCCESS` yazana kadar bekleyecektir.
```bash
slam
```

### 4. PC — Terminal: Görselleştirme
PC tarafında haritayı ve robotun durumunu RViz üzerinden izleyin.
```bash
rvizslam
```

### 5. Haritayı Kaydetme
Gezme işlemi bittiğinde Raspi'de yeni bir terminal açarak:
```bash
savemap
```
*(Dosyalar `~/REALNAV2.3-/maps/` altına kaydedilir)*

---

## 🛑 Durdurma Sırası

### 1. SLAM Kapatma
Slam terminalinde **Ctrl+C** yapın. Yazdığımız `start_rplidar.sh` scripti motoru otomatik olarak durduracaktır.

### 2. Teleop Kapatma
Teleop terminalinde **Q** tuşuna basın veya **Ctrl+C** yapın.

### 3. Acil Temizlik
Eğer LiDAR motoru hala dönüyorsa veya bir şeyler asılı kaldıysa Raspi'de şu komutu çalıştırın:
```bash
killrobot
```
*(Bu komut tüm ROS process'lerini öldürür ve motoru serial üzerinden zorla durdurur)*

---

## 🛠️ Yararlı Kısayollar (Aliases)

| Komut | Açıklama |
| :--- | :--- |
| `slam` | SLAM + LiDAR Launch başlatır (Raspi) |
| `teleop` | Klavye kontrol (WASD) + Odom başlatır (Raspi) |
| `savemap` | Haritayı `/maps` klasörüne kaydeder (Raspi) |
| `killrobot` | Her şeyi öldürür ve motoru durdurur (Raspi) |
| `rvizslam` | RViz'i yapılandırılmış halde açar (PC) |
| `getmap` | Haritayı Raspi'den PC'ye indirir (PC) |
| `buildrobot` | Projeyi derler (Raspi) |

---

> [!IMPORTANT]  
> Robotu sürerken (WASD) dönüşleri çok hızlı yapmamaya çalışın. LiDAR'ın haritayı "oturtması" için yavaş sürüş (PWM 60-80) tavsiye edilir.
