# SeraBot – Tasarım ve Süreç Notları

Bu dosya, raporun "görev, özgün tasarım, hareket sistemi, bileşen bağlantıları ve sensörler" bölümleri için kaynak olarak hazırlandı. Projenin ilk yarısını (altyapı, robot modeli, dünya, LiDAR, IMU entegrasyonu) **Metehan**, ikinci yarısını (kamera, IMU görselleştirme, davranış düğümü, rapor) **Samet** üstlenmiştir.

---

## 1. Ekip ve süreç

| Tarih | Aşama | Sorumlu |
|---|---|---|
| 26 – 30 Ekim | Planlama | Metehan ve Samet birlikte |
| 30 Ekim – 2 Kasım | Birinci yarı: altyapı, model, dünya, LiDAR, IMU entegrasyonu | Metehan |
| 2 Kasım | Devir teslim | Metehan → Samet |
| 2 – 7 Kasım | İkinci yarı: kamera, IMU görselleştirme, davranış düğümü, README, rapor, video | Samet |
| 8 Kasım, 23:59 | Rapor teslimi (CANVAS) | Samet (Metehan kontrol eder) |
| 9 Kasım, 11:10 | Sunum ve canlı demo | Metehan ve Samet |

### 1.1 Planlama aşaması (26 – 30 Ekim)
- Ödev şartları incelendi: özgün robot, URDF/Xacro modeli, Gazebo'da `/cmd_vel` ile sürüş, tek launch dosyası, 2 kişilik grup için **en az 3 farklı sensör tipi**, her üyenin en az bir sensörden sorumlu olması ve sensör verisine göre davranış belirleyen bir ROS 2 düğümü.
- Üç robot fikri karşılaştırıldı:

  | Fikir | Artısı | Eksisi |
  |---|---|---|
  | **SeraBot** (seçilen) | Somut ve özgün senaryo; üç sensörün de görevde anlamlı bir işi var; davranış iki seviyeli kurulabiliyor | – |
  | KütüpBot (raf arası duvar takibi) | Görsel olarak etkileyici | Duvar takibinin ayarı engelden kaçmaya göre daha zor |
  | MadenKaşifi (skid-steer tünel robotu) | Çok özgün, IMU kullanımı en anlamlı | Skid-steer ve eğimli zemin daha fazla ayar gerektiriyor, risk yüksek |

- SeraBot seçildi, çünkü riski en düşük olan zorunlu davranış (LiDAR ile engelden kaçma) garantili çalışacak bir temel sağlıyor; kamera ile kırmızı bitkiye dönme ise bonus olarak eklenebiliyor.
- İş bölümü sensör bazında yapıldı (bkz. Bölüm 7): LiDAR Metehan'da, kamera Samet'te, ekstra sensör olan IMU ortak (entegrasyon Metehan, okuma ve görselleştirme Samet).
- Projenin planı, görevleri ve devir teslim kontrol listesi `SeraBot_Proje_Plani.pdf` belgesinde toplandı.

### 1.2 Birinci yarı (30 Ekim – 2 Kasım, Metehan)
Her adım ayrı commit'ler hâlinde ilerledi:

1. **Repo ve workspace:** GitHub'da public repo, repo kökü aynı zamanda colcon workspace'i; `serabot_description`, `serabot_gazebo`, `serabot_behavior` paketleri.
2. **Robot modeli:** eylemsizlik makroları, parametrik Xacro modeli, RViz'de görüntüleme için `display.launch.py` ve `display.rviz`. Düz silindir tekerleklerin dönüşü gözle görülmediği için tekerleklere görsel bir şerit eklendi.
3. **Sera dünyası:** iskelet → sensör sistem plugin'leri → duvarlar → 3 saksı sırası (her biri ayrı commit) → gübre çuvalları → kırmızı hasta bitki.
4. **Sürüş:** diff-drive ve joint state publisher plugin'leri, `ros_gz_bridge` ayarları, tek komutluk `sim.launch.py` ve `sim.rviz`.
5. **LiDAR:** `gpu_lidar` sensörü → bridge'e `/scan` → RViz'e LaserScan display'i.
6. **IMU:** IMU sensörü → bridge'e `/imu`.
7. **Dokümantasyon:** README (kurulum, çalıştırma, konu listesi) ve bu notlar.

### 1.3 İkinci yarı (2 – 7 Kasım, Samet)
Kamera ve optik frame, `/camera/image_raw` ile `/camera/camera_info` köprüleri, rqt_plot ile IMU grafikleri, `obstacle_avoider.py` davranış düğümü, `behavior:=true` launch argümanı, simülasyon ekran görüntüleri, yedek demo videosu ve rapor. Ayrıntılar için Bölüm 8'deki devir notlarına bakın.

---

## 2. Görev

SeraBot, sera koridorlarında devriye gezen bir bitki gözlem robotudur. Saksı sıralarının arasında sıralara ve duvarlara çarpmadan ilerler. Kamerasıyla kırmızı renkli "hasta" bitkiyi fark ettiğinde ona doğru dönüp durur; bu an bitkinin raporlanması olarak yorumlanır.

---

## 3. Tasarım gerekçesi

### 3.1 Hareket sistemi: diferansiyel sürüş
- **2 tahrikli tekerlek + arkada 1 küresel caster.** Dar koridorlarda yerinde dönebilmek için en uygun ve en basit sürüş tipi; Gazebo'da hazır `gz-sim-diff-drive-system` plugin'i ile doğrudan desteklenir.
- Tekerlekler gövdenin **ön yarısında** (x = +0.10 m). Ağırlık tekerlekler ile arka caster arasında dağılır; caster sürtünmesiz tanımlandığı için dönüşleri frenlemez.
- Skid-steer alternatifi değerlendirildi ancak tekerlek kayması nedeniyle simülasyonda daha fazla ayar gerektirdiği için tercih edilmedi.

### 3.2 Boyutlar
| Parça | Değer | Gerekçe |
|---|---|---|
| Gövde | 0.45 × 0.30 × 0.12 m, yeşil | İnce ve uzun şase: ~1 m'lik koridorlarda iki yanda pay kalır |
| Zemin boşluğu | 0.04 m | Caster küresinin çapına eşit |
| Tekerlek | r = 0.07 m, genişlik 0.04 m, siyah | Gövde dışında, y = ±0.175 m; tekerlek aralığı 0.35 m |
| Caster | Küre, r = 0.02 m, x = −0.17 m | Hem zemine hem gövdenin altına değer |
| Direk | r = 0.015 m, h = 0.25 m, x = −0.15 m | Kamerayı bitki yüksekliğine taşır |
| Durum lambası | Turuncu silindir, r = 0.025 m, h = 0.03 m | Yalnızca görsel; üstü LiDAR düzleminin altında kalır |
| Toplam genişlik | 0.39 m (tekerlekler dahil) | Koridorda iki yanda ~0.3 m boşluk |

Tüm boyutlar `serabot.urdf.xacro` içinde `xacro:property` olarak tanımlıdır; tekerlek eklemlerinin yüksekliği, diff-drive plugin'inin tekerlek aralığı ve yarıçapı bu parametrelerden hesaplanır.

### 3.3 Sensör yerleşimi
| Sensör | Link | Konum | Gerekçe |
|---|---|---|---|
| LiDAR | `lidar_link` | Gövdenin ön üstünde, 0.03 m'lik kaide üzerinde; tarama düzlemi zeminden **0.21 m** | Saksılar (0.30 m), çuvallar (0.25 m) ve duvarlar (1 m) bu yüksekliği keser |
| Kamera | `camera_link` | Direğin tepesinde, zeminden ~0.41 m, **10° aşağı** bakıyor | Bitki kürelerini (merkez 0.45 m) ve önündeki koridoru birlikte görür |
| IMU | `imu_link` | Gövde merkezinde | Ağırlık merkezine yakın; dönüş sırasında merkezkaç etkisi en az |

**Bilinen kısıt:** Direk LiDAR'ın tarama düzlemini keser. LiDAR'ın tam arkasında (~180°, ~0.28 m mesafede) yaklaşık 6°'lik dar bir kör bölge oluşur. Davranış düğümü yalnızca ön ve yan sektörleri kullandığı için pratik etkisi yoktur.

---

## 4. Robot modeli ve bileşen bağlantıları

### 4.1 Dosyalar
| Dosya | İçerik |
|---|---|
| `serabot_description/urdf/serabot.urdf.xacro` | Parametreler, link ve eklemler; diğer dosyaları include eder |
| `serabot_description/urdf/inertial_macros.xacro` | Kutu, silindir ve küre için eylemsizlik makroları |
| `serabot_description/urdf/serabot_gazebo.xacro` | Diff-drive ve joint state publisher plugin'leri, sürtünme ayarları |
| `serabot_description/urdf/serabot_sensors.xacro` | LiDAR (`gpu_lidar`) ve IMU sensörleri |

### 4.2 TF ağacı
```
odom                         (diff-drive plugin'i yayınlar)
└── base_footprint           (zemin, robotun izdüşümü)
    └── base_link            (gövde merkezi, zeminden 0.10 m)
        ├── left_wheel_link  (continuous, y ekseni)
        ├── right_wheel_link (continuous, y ekseni)
        ├── caster_link      (fixed)
        ├── lidar_link       (fixed)
        ├── imu_link         (fixed)
        ├── lamp_link        (fixed)
        └── mast_link        (fixed)
            └── camera_link  (fixed, pitch 10°)
```
Rapor için TF ağacının PDF'i: `ros2 run tf2_tools view_frames`.

---

## 5. Simülasyon dünyası (`serabot_gazebo/worlds/greenhouse.sdf`)

| Öğe | Ayrıntı |
|---|---|
| Sera | 10 × 6 m iç alan (x: −5…5, y: −3…3), 1 m yüksek duvarlar, toprak renkli zemin |
| Saksı sıraları | y = −1.4, 0, 1.4; her sırada x = −3…3 arasında 7 saksı (kahverengi silindir r = 0.15 m, h = 0.30 m + yeşil küre r = 0.18 m) |
| Koridorlar | İç koridorlar ~1.0 m, dış koridorlar ~1.4 m, sıra uçlarında ~1.8 m'lik dönüş alanları |
| Gübre çuvalları | 4 statik kutu (0.5 × 0.35 × 0.25 m) |
| Hasta bitki | `plant_row_2 / pot_4`, **x = 1.0, y = 1.4**, kırmızı (0.85, 0.05, 0.05) |
| Sistem plugin'leri | Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), Imu |
| Başlangıç konumu | (−4.0, 0.7), yaw 0: batı dönüş alanı, orta koridora bakıyor |

---

## 6. ROS 2 konuları ve mesaj tipleri

| Konu | Mesaj tipi | Yön | Hız | Frame |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS | 10 Hz | `lidar_link` |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS | 100 Hz | `imu_link` |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS → Gazebo | – | – |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | 30 Hz | `odom` → `base_footprint` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | – | `odom` → `base_footprint` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | – | – |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | – | – |

**Sensör ayarları**
- **LiDAR (`gpu_lidar`):** 360°, 360 örnek (1° çözünürlük), 10 Hz, menzil 0.12–10 m, Gauss gürültüsü σ = 0.01 m.
- **IMU:** 100 Hz; jiroskop gürültüsü σ = 0.0002 rad/s, ivmeölçer gürültüsü σ = 0.017 m/s².

**Neden `TwistStamped`:** Lyrical'daki TurtleBot3 simülasyonunun bridge yapılandırması da `/cmd_vel` için `TwistStamped` kullanıyor; projede aynı yaklaşım benimsendi. Bu yüzden teleop `-p stamped:=true` ile çalıştırılır.

### 6.1 Doğrulama sonuçları
| Kontrol | Sonuç |
|---|---|
| `/cmd_vel` 0.3 m/s, ~5 s | `/odom` x ≈ 1.4–1.7 m; robot ilerliyor |
| `/cmd_vel` 0.5 rad/s, 3 s | ≈108° dönüş |
| `ros2 topic hz /scan` | 9.99 Hz, `frame_id: lidar_link` |
| LiDAR ölçümleri (başlangıç konumunda) | Ön 7.89 m (koridor sonundaki çuval), sol 2.30 m ve sağ 3.70 m (duvarlar), ±45° ~1.0 m (ilk saksılar) |
| `ros2 topic hz /imu` | 100.02 Hz, `frame_id: imu_link` |
| IMU dururken | `linear_acceleration.z` ≈ 9.81 m/s² (yerçekimi) |
| IMU 0.8 rad/s dönüş komutunda | `angular_velocity.z` ≈ 0.90 rad/s |

---

## 7. Katkı tablosu

| Sensör / İş | Entegrasyon | Veri okuma | Görselleştirme |
|---|---|---|---|
| LiDAR | Metehan | Metehan | Metehan (RViz) |
| Kamera | Samet | Samet | Samet (rqt_image_view) |
| IMU (ekstra, ortak) | Metehan | Samet | Samet (rqt_plot) |
| Davranış düğümü | Samet | Samet | Samet |
| Altyapı, model, dünya, launch | Metehan | – | – |

---

## 8. Samet için devir notları

- **Build:** Yeni bir dosya ekledikten sonra `colcon build --symlink-install` çalıştırılmalı; aksi hâlde launch `No such file or directory` hatası verir. Var olan dosyalardaki değişiklikler için build gerekmez.
- **Kamera:** `camera_link` hazır (direğin tepesinde, pitch 10°). Sensör `serabot_sensors.xacro` içine eklenebilir; optik frame (`camera_optical_link`) için `camera_link`'e `rpy="-π/2 0 -π/2"` ile bağlı bir link gerekir. Bridge girdileri `serabot_gazebo/config/bridge.yaml` dosyasının sonuna eklenir.
- **Davranış düğümü çıkışı:** `/cmd_vel` **`geometry_msgs/msg/TwistStamped`** tipindedir. `Twist` yayınlanırsa bridge mesajları almaz.
- **LiDAR açıları:** `angle_min = −π`, `angle_increment` ≈ 1° (açı = `angle_min + i · angle_increment`, sabit değer yerine mesajdaki alanları kullanın); indeks 180 robotun önüdür (0°), indeks 270 sol (+90°), indeks 90 sağ (−90°). Ön sektör (±30°) indeks 150–210 arası.
- **Arka kör bölge:** İndeks ~0 ve ~359 civarında direk ~0.28 m olarak görünür; arka sektör kullanılmamalı.
- **Hasta bitki:** x = 1.0, y = 1.4 (kuzey sıra). Orta koridordan (y = 0.7) ve kuzey dış koridordan görülebilir.
- **IMU grafikleri:** rqt_plot'ta `/imu/angular_velocity/z` ve `/imu/linear_acceleration/x`.
- **Launch:** `behavior:=true` argümanı `serabot_gazebo/launch/sim.launch.py` içine eklenecek; diğer argümanlar `world`, `x`, `y`, `yaw`, `rviz`.
- **Arka planda kalan Gazebo:** `pgrep -af gz`, gerekirse `pkill -f gz-sim-main`.

---

## 9. Ekran görüntüleri

| Dosya | İçerik |
|---|---|
| `docs/model_asamalari/model_1.png` | Robot modeli, RViz |
| `docs/model_asamalari/model_2.png` | Tekerlek işaretli model |
| `docs/model_asamalari/model_serada.png` | Robot Gazebo'da, sera içinde |
| `docs/model_asamalari/lidar_lazer.png` | LiDAR verisi |
| `docs/dunya_asamalari/sera_zemin.png` … `sera_hastalikli_bitki.png` | Dünyanın aşama aşama oluşumu |
