## REALNAV2 — SLAM & Navigation Sistemi (Raspberry Pi 5 + STM32 + RPLIDAR A2M12)

Bu proje, Raspberry Pi 5 ve STM32 tabanlı gerçek bir robot için ROS 2 Jazzy üzerinde:

- **SLAM (eşzamanlı konumlandırma ve haritalama)**
- **Nav2 ile otonom navigasyon**

sağlar. LiDAR (RPLIDAR A2M12) **ayrı bir terminalde** çalışır; SLAM ve Nav2 launch dosyaları LiDAR’ı asla yeniden başlatmaz. Böylece USB + harici 5V güç senaryosunda LiDAR’ın kilitlenme riskini minimuma indiririz.

---

## 🛜 Ağ ve Ortam Ayarları

- **Raspi IP:** `192.168.68.111`  (örnek)
- **PC IP:** `192.168.68.118`    (örnek; kendi IP’nize göre değişir)
- **Domain ID:** `0`
- **DDS:** FastDDS, XML profil dosyaları (`fastdds_pc.xml`, `fastdds_raspi.xml`)

### Tek Seferlik Kurulum (PC + Raspi)

- Hem PC hem Raspi’de `.bashrc` sonuna:

**PC (`~/.bashrc`):**

```bash
source /path/to/REALNAV4.0/pc_env.sh
```

**Raspi (`~/.bashrc`):**

```bash
source /path/to/REALNAV4.0/robot_env.sh
```

`pc_env.sh` ve `robot_env.sh` kendi bulundukları dizini otomatik olarak `REALNAV_ROOT` kabul eder. Yani:

- PC için genelde: `REALNAV_ROOT = ~/Desktop/REALNAV4.0`
- Raspi için genelde: `REALNAV_ROOT = ~/REALNAV4.0`

Bu sayede her iki tarafta da path hardcode etmeye gerek kalmaz.

---

## 🔐 RPLIDAR A2M12 — Tek Seferlik USB İzinleri

RPLIDAR A2M12, genellikle **CP2102 / 10c4:ea60** USB–seri dönüştürücü üzerinden `/dev/ttyUSB0` olarak görünür.

### 1. Kullanıcıyı `dialout` grubuna ekle

Raspi’de:

```bash
sudo usermod -a -G dialout $USER
```

Ardından **logout/login** yap (veya sistemi reboot et), sonra:

```bash
groups
```

çıktısında `dialout` görmelisin.

### 2. Udev kuralı ile kalıcı izin ver

Raspi’de:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Sonra USB kablosunu bir tak–çıkar yap veya sistemi yeniden başlat. Bundan sonra:

```bash
ls -l /dev/ttyUSB0
```

çıktısında `crw-rw-rw-` (veya en azından kullanıcı için `rw-`) görmelisin. Böylece LiDAR script’i **sudo gerektirmeden** çalışabilir.

---

## ⚙️ Projeyi Güncelleme ve Derleme (Raspi)

### 1. PC’den Raspi’ye kod senkronizasyonu

Önerilen yöntem `rsync` ile **sadece kaynak kodu** taşımak:

PC’de:

```bash
rsync -av --delete \
  --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
  --exclude='**/__pycache__' --exclude='**/*.pyc' \
  ~/Desktop/REALNAV4.0/ raspi@192.168.68.111:~/REALNAV4.0/
```

Alternatif olarak ilk kurulumda:

```bash
scp -r ~/Desktop/REALNAV4.0 raspi@192.168.68.111:~/
```

### 2. Raspi’de derleme

Raspi’de:

```bash
cd ~/REALNAV4.0
rm -rf build install log          # Eski derlemeyi temizle
colcon build --symlink-install
```

Derleme bittikten sonra (yeni bir terminalde veya aynı terminalde):

```bash
source ~/REALNAV4.0/robot_env.sh
```

`robot_env.sh` şunları yapar:

- ROS 2 Jazzy ortamını (`/opt/ros/jazzy/setup.bash`) yükler.
- Proje `install` ortamını (`$REALNAV_ROOT/install/setup.bash`) yükler.
- RPLIDAR ve Nav2 için gerekli alias’ları tanımlar (`lidar`, `slam`, `nav2`, `teleop`, `savemap`, `killrobot`…).

---

## 🧠 Kavramsal Harita — Hangi terminal ne yapıyor?

Gerçek robotta üç ana “taraf” var:

- **LiDAR tarafı**: `/scan` topic’ini üretir.
- **Odometry / STM32 tarafı**:
  - `teleop` veya `nav2_motor_bridge.py` üzerinden STM32’ye PWM gönderir.
  - Encoder’dan `odom → base_footprint` TF ve `/odom` yayınlar.
- **SLAM + Nav2 tarafı**:
  - SLAM Toolbox: `map → odom` TF ve `/map` yayınlar.
  - Nav2: global/local costmap, planner, controller, bt_navigator, collision_monitor vs.

LiDAR **her zaman AYRI bir terminalde** çalışır. SLAM ve Nav2 launch dosyaları LiDAR’ı ne başlatır ne de durdurur; sadece `/scan` topic’ini kullanır.

---

## 🚀 Çalıştırma Sırası (Detaylı Senaryo)

### 0. Donanım Hazırlığı

- STM32’nin gücünü kesin ve tekrar takın (Power Reset).
- RPLIDAR’a:
  - USB kablosu ile veri bağlantısı,
  - Ayrı 5V güç (gerekiyorsa) verildiğinden emin olun.

> **Not:** Harici 5V verdiğin için, USB tarafı reset atsa bile motor bir süre daha dönebilir. Bizim script’ler **LiDAR protokol komutları** ile motoru durdurmaya çalışır; yine de kablo teması ve güç beslemesi kritik.

---

### 1. Raspi — Terminal 1: LiDAR (HER ŞEYİN TEMELİ)

Bu terminal sadece LiDAR’a ayrılmıştır.

```bash
cd ~/REALNAV4.0
source robot_env.sh      # Eğer yeni terminal açtıysan
lidar
```

Bu komut:

- `src/my_robot_bringup/scripts/start_rplidar.sh` script’ini çalıştırır.
- Script’in yaptığı başlıca işler:
  - `/dev/ttyUSB0` var mı diye **20 saniyeye kadar bekler**.
  - Gerekirse port izinlerini düzeltir (udev + dialout doğru ise genelde gerek kalmaz).
  - Eski `rplidar_composition` process’lerini öldürür.
  - `ros2 run rplidar_ros rplidar_composition ...` ile node’u başlatır.
  - Birkaç saniye sonra node’un gerçekten ayakta olduğundan emin olur.
  - Başarılıysa:

    ```text
    [LiDAR] Çalışıyor (PID=XXXX). /scan yayınını kontrol edebilirsin.
    ```

    mesajını yazdırır.
  - SIGINT/SIGTERM (Ctrl+C) aldığında:
    - `rplidar_composition` process’ini öldürür.
    - Protokol komutları ile motoru durdurmaya çalışır:
      - `0xA5 0x25` (STOP_SCAN)
      - `0xA5 0xF0 0x02 0x00 0x00 0x57` (SET_MOTOR_PWM(0))

#### LiDAR’ın gerçekten çalıştığını nasıl anlarsın?

İkinci bir Raspi terminalinde:

```bash
source ~/REALNAV4.0/robot_env.sh
ros2 topic echo /scan --once
```

Bir adet `sensor_msgs/msg/LaserScan` mesajı görüyorsan (örnek olarak daha önce paylaştığın çıktıya benziyorsa), LiDAR yazılımsal olarak sağlıklı çalışıyor demektir.

---

### 2. Raspi — Terminal 2: SLAM (Haritalama Modu)

Bu terminal, gerçek zamanda harita çıkarma (SLAM) içindir. Burada `wasd_teleop.py` ile robotu manuel sürerken, SLAM Toolbox haritayı oluşturur.

```bash
cd ~/REALNAV4.0
source robot_env.sh
slam
```

Bu alias şu launch dosyasını çalıştırır:

- `my_robot_bringup/launch/slam_teleop.launch.py`

Launch dosyasının yaptığı:

- `robot_state_publisher`:
  - URDF dosyasından (`my_robot.urdf.xacro`) robotun TF ağacını üretir:
    - `base_link`, `laser_link`, tekerlek link’leri, vb.
- `joint_state_publisher`:
  - Tekerlek joint durumlarını (statik veya fake) yayınlar; TF zincirini tamamlar.
- SLAM Toolbox (online_async):
  - `/scan` ve `/odom` kullanarak:
    - `map → odom` TF’ini yayınlar.
    - `/map` topic’ini üretir.

> **ÖNEMLİ:** `slam_teleop.launch.py` **LiDAR’ı BAŞLATMAZ ve DURDURMAZ**. `/scan`’i hazır bulur ve kullanır. Bu yüzden önce **Terminal 1’de `lidar`** çalışıyor olmalı.

Ayrı bir terminalde (Terminal 3, aşağıda) `teleop` ile:

- Klavye (WASD) ile robotu sürersin.
- `wasd_teleop.py` STM32 ile konuşarak:
  - Sol/sağ tekerlek PWM komutlarını gönderir.
  - Encoder verisinden `/odom` ve `odom → base_footprint` TF’ini hesaplar.

---

### 3. Raspi — Terminal 3: Teleop (Manuel Sürüş + Odometry)

```bash
cd ~/REALNAV4.0
source robot_env.sh
teleop
```

Bu alias:

- `src/my_robot_bringup/scripts/wasd_teleop.py` script’ini çalıştırır.

Script:

- `W, A, S, D, SPACE, +, -` tuşları ile sol/sağ PWM komutları gönderir.
- STM32’den gelen encoder verilerini okuyup:
  - Pozisyonu (`x, y, yaw`) entegre eder.
  - `odom → base_footprint` TF’ini yayınlar.
  - `/odom` topic’ini yayınlar.

> **Önerilen PWM aralığı:** Başlangıç için 60–80 arası. Çok hızlı dönüşler SLAM haritasını bozabilir.

---

### 4. PC — Görselleştirme (SLAM Modu)

PC’de:

```bash
cd ~/Desktop/REALNAV4.0
source pc_env.sh
rvizslam
```

Bu alias:

- Rviz’i SLAM görünümü ile açar (`exploration.rviz`).
- `/map`, `/odom`, `/scan`, TF ve robot modelini görürsün.

Haritalama bittikten sonra Raspi’de yeni bir terminal aç:

```bash
cd ~/REALNAV4.0
source robot_env.sh
savemap
```

- Harita dosyaları `REALNAV4.0/maps/` altına kaydedilir.

PC’ye almak için:

```bash
cd ~/Desktop/REALNAV4.0
source pc_env.sh
getmap
```

---

## 🧭 Kayıtlı Haritayla Nav2 (Otonom Navigasyon)

Nav2 tarafı, önceden kaydedilmiş bir harita (`my_room_map.yaml` vb.) ile çalışır.

### 1. Raspi — Terminal 1: LiDAR (DEĞİŞMEZ)

Her zaman olduğu gibi:

```bash
cd ~/REALNAV4.0
source robot_env.sh
lidar
```

### 2. Raspi — Terminal 2: Odom / Motor Köprüsü (Nav2 Modu)

Nav2 modunda odometry, `nav2_motor_bridge.py` üzerinden sağlanır (ros2_control olmadan doğrudan STM32 ile konuşan köprü).

Bu köprü `real_robot_nav2.launch.py` tarafından otomatik başlatılır; ayrıca bir komut girmen gerekmez.

### 3. Raspi — Terminal 3: Nav2 (kayıtlı harita)

```bash
cd ~/REALNAV4.0
source robot_env.sh
nav2
```

Bu alias:

- `my_robot_bringup/launch/real_robot_nav2.launch.py` dosyasını çalıştırır.
- Argüman olarak:
  - `map:="$REALNAV_ROOT/maps/my_room_map.yaml"` verir (alias içinde).

Launch dosyasının yaptığı:

- `robot_state_publisher` + `joint_state_publisher`:
  - URDF + TF ağacını kurar (`base_link`, `laser_link`, tekerlekler…).
- `nav2_motor_bridge.py`:
  - `/cmd_vel` → STM32 PWM komutları (`/dev/ttyAMA0`).
  - Encoder’dan `/odom` + `odom → base_footprint` TF’i.
- LiDAR:
  - **Ayrı terminalde çalışan** `lidar` üzerinden `/scan` sağlar.
- `nav2_map_server` + `amcl`:
  - `map.yaml`’dan statik haritayı yükler.
  - `map → odom` TF’ini (AMCL) yayınlar.
- Nav2 core node’ları:
  - `controller_server`, `planner_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `collision_monitor`…
- Lifecycle yöneticileri:
  - Localization ve navigation node’larını doğru sırayla `configure` + `activate` eder.

> **Kritik:** Nav2’yi açmadan önce:
> - LiDAR (`lidar`) çalışıyor olmalı.
> - Odom TF’i (STM32 + `nav2_motor_bridge.py`) düzgün gelmeli.

### 4. PC — RViz (Nav2 Modu)

PC’de:

```bash
cd ~/Desktop/REALNAV4.0
source pc_env.sh
rviznav2
```

- Nav2 için hazırlanmış RViz config’i ile:
  - Global/local costmap, path, TF, robot modeli ve haritayı görürsün.
  - Nav2 panelinden goal gönderip robotu otonom sürersin.

---

## 🛑 Durdurma Sırası (Güvenli Kapatma)

1. **Nav2 Terminali (varsa) — Ctrl+C**
   - `real_robot_nav2.launch.py` kapanır.

2. **SLAM Terminali (varsa) — Ctrl+C**
   - `slam_teleop.launch.py` kapanır.

3. **Teleop Terminali (varsa)**
   - `Q` tuşuna bas veya **Ctrl+C** yap.

4. **LiDAR Terminali (HER ZAMAN EN SON) — Ctrl+C**
   - `start_rplidar.sh` cleanup fonksiyonu:
     - `rplidar_composition` process’ini öldürür.
     - Motoru durdurma komutlarını (SET_MOTOR_PWM(0)) gönderir.

5. **Acil Durum (bir şeyler sapıttıysa):**

Raspi’de yeni bir terminal aç:

```bash
cd ~/REALNAV4.0
source robot_env.sh
killrobot
```

Bu komut:

- Tüm ilgili ROS process’lerini öldürmeye çalışır.
- RPLIDAR motorunu protokol komutları ile zorla durdurur.

---

## 🛠️ Yararlı Kısayollar (Aliases) — Özet Tablo

### Raspi (robot_env.sh)

| Komut      | Açıklama |
| :---       | :--- |
| `lidar`    | RPLIDAR A2M12’i `start_rplidar.sh` ile başlatır (ayrı terminal). |
| `slam`     | Sadece SLAM + TF (LiDAR’ın `/scan` topic’ini kullanır). |
| `teleop`   | WASD klavye kontrol + `/odom` + `odom → base_footprint` TF. |
| `nav2`     | Kayıtlı haritayla Nav2’yi başlatır (`real_robot_nav2.launch.py`). |
| `slamnav2` | SLAM + Nav2 için kombine launch (ileri seviye kullanım, isteğe bağlı). |
| `savemap`  | Haritayı `REALNAV4.0/maps/` altına kaydeder. |
| `buildrobot` | Projeyi derler (`colcon build --symlink-install`). |
| `killrobot`  | Tüm robot process’lerini öldürür + LiDAR motorunu durdurmaya çalışır. |

### PC (pc_env.sh)

| Komut      | Açıklama |
| :---       | :--- |
| `rvizslam` | SLAM görünümü ile RViz2’yi açar. |
| `rviznav2` | Nav2 görünümü ile RViz2’yi açar. |
| `getmap`   | Raspi’den harita dosyalarını indirir. |
| `buildpc`  | PC tarafında projeyi derler. |

---

## 🚶‍♂️ Pratik Örnek Senaryolar

### A) Sıfırdan Harita Çıkarma (SLAM)

1. Raspi — Terminal 1:

```bash
cd ~/REALNAV4.0
source robot_env.sh
lidar
```

2. Raspi — Terminal 2:

```bash
cd ~/REALNAV4.0
source robot_env.sh
slam
```

3. Raspi — Terminal 3:

```bash
cd ~/REALNAV4.0
source robot_env.sh
teleop
```

4. PC:

```bash
cd ~/Desktop/REALNAV4.0
source pc_env.sh
rvizslam
```

5. Haritayı kaydet:

```bash
cd ~/REALNAV4.0
source robot_env.sh
savemap
```

---

### B) Kayıtlı Harita ile Otonom Navigasyon (Nav2)

Ön koşul: `REALNAV4.0/maps/` içinde çalışma odasına ait bir `my_room_map.yaml` (ve `.pgm`/`.png`) kaydedilmiş olmalı.

1. Raspi — Terminal 1:

```bash
cd ~/REALNAV4.0
source robot_env.sh
lidar
```

2. Raspi — Terminal 2:

```bash
cd ~/REALNAV4.0
source robot_env.sh
nav2
```

3. PC:

```bash
cd ~/Desktop/REALNAV4.0
source pc_env.sh
rviznav2
```

4. RViz içinden:
   - Nav2 panelinden goal seç.
   - Robotun harita üzerinde otonom gittiğini izle.

---

> [!IMPORTANT]  
> Robotu sürerken (WASD veya Nav2) özellikle dar alanlarda **yavaş ve kontrollü** hareket ettir. LiDAR + SLAM kombinasyonunun iyi harita üretmesi için ani dönüşlerden kaçınmak, düz ve yavaş sürüşler yapmak en sağlıklısıdır.*** End Patch
