# 🐾 Go2 Nav Demos

Unitree **Go2** robot için ROS 2 tabanlı geometrik navigasyon demoları.  
Robot, Gazebo simülasyonunda farklı yörüngeleri **odometry geri beslemeli** kapalı döngü kontrolle izler.

---

## 📺 Demo Videosu

https://github.com/USER/go2_nav_demos/raw/main/media/kare_tur.mp4

> **Kare Tur** — Robot 5×5 m kare alanda 3 tur atar.

---

## 🗂️ Repo Yapısı

```
go2_nav_demos/
├── worlds/
│   ├── kare_world.world      # Gazebo 5×5 m kare alan (sarı duvarlar)
│   ├── daire_world           # Gazebo daire alan (sarı işaretleyiciler)
│   ├── elips.world           # Gazebo elips alan (sarı işaretleyiciler)
│   └── sonsuzluk_world       # Gazebo lemniskat (∞) alan (sarı işaretleyiciler)
├── scripts/
│   ├── kare_tur.py           # ✅ Kare yörünge (tamamlandı)
│   ├── daire_tur.py          # ✅ Daire yörünge (tamamlandı)
│   ├── elips_tur.py          # ✅ Elips yörünge (tamamlandı)
│   └── sonsuzluk_tur.py      # ✅ Sonsuzluk (∞ / lemniskat) yörüngesi (tamamlandı)
├── media/
│   └── kare_tur.mp4          # Demo video
└── README.md
```

---

## 🚀 Kurulum & Çalıştırma

### Gereksinimler

- ROS 2 Humble (veya üzeri)
- Gazebo Classic veya Ignition
- `geometry_msgs`, `nav_msgs` paketleri
- Unitree Go2 ROS 2 sürücüsü (veya `go2_description`)

### Simülasyonu Başlat

```bash
# Gazebo dünyasını aç (istediğin yörüngeye göre world dosyasını seç)
ros2 launch gazebo_ros gazebo.launch.py \
  world:=$(pwd)/worlds/kare_world.world      # veya: daire_world / elips.world / sonsuzluk_world

# Robotu spawn et (ayrı terminal)
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity go2
```

### Scriptleri Çalıştır

```bash
# Kare tur
python3 scripts/kare_tur.py

# Daire tur
python3 scripts/daire_tur.py

# Elips tur
python3 scripts/elips_tur.py

# Sonsuzluk (∞) turu
python3 scripts/sonsuzluk_tur.py
```

> Her script kendi içinde `nav_msgs/Odometry` mesajını **`/odom/ground_truth`** topiğinden dinler ve `geometry_msgs/Twist` komutlarını **`/cmd_vel`** topiğine yayınlar.

---

## 📐 Yörüngeler

### ■ Kare Tur (`kare_tur.py`)

Robot, 5×5 m karenin 4 köşesini sırayla ziyaret eder.

```
(-2.5, 2.5) ──────── (2.5, 2.5)
     │                    │
     │                    │
(-2.5,-2.5) ──────── (2.5,-2.5)  ← başlangıç
```

| Parametre | Değer |
|-----------|-------|
| Alan | 5×5 m |
| Tur sayısı | 3 |
| İleri hız | 0.4 m/s (max, mesafeye oransal) |
| Hedef toleransı | 0.2 m |

---

### 〇 Daire Turu (`daire_tur.py`)

Robot, yarıçapı 3 m olan daireyi 12 waypoint ile, durmadan akıcı geçişle izler.

```
x(t) = r·cos(t)
y(t) = r·sin(t)        t ∈ [0, 2π)
```

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `r` | 3.0 m | Yarıçap |
| `n` | 12 | Waypoint sayısı (az = daha akıcı geçiş) |
| Tur sayısı | 3 | |
| İleri hız | 0.35 m/s (sabit) | |
| Hedef toleransı | 0.5 m (başlangıçta 0.3 m) | |

---

### 〇 Elips Turu (`elips_tur.py`)

Robot, parametrik elips yörüngesini 24 waypoint ile izler.

```
x(t) = a·cos(t)
y(t) = b·sin(t)        t ∈ [0, 2π)
```

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `a` | 4.0 m | X yarı eksen (toplam genişlik 8 m) |
| `b` | 2.5 m | Y yarı eksen (toplam yükseklik 5 m) |
| `N` | 24 | Waypoint sayısı |
| Tur sayısı | 3 | |
| İleri hız | 0.4 m/s (max, mesafeye oransal) | |
| Hedef toleransı | 0.25 m | |

---

### ♾️ Sonsuzluk Turu (`sonsuzluk_tur.py`)

Robot, **Bernoulli Lemniskatı** yörüngesini izler — ortadan geçen sekiz şekli.

```
x(t) = a·cos(t) / (1+sin²(t))
y(t) = a·sin(t)·cos(t) / (1+sin²(t))       t ∈ [0, 2π)
```

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `a` | 3.5 m | Ölçek katsayısı |
| `N` | 32 | Waypoint sayısı |
| Tur sayısı | 3 | |
| İleri hız | 0.4 m/s (max, mesafeye oransal) | |
| Hedef toleransı | 0.35 m (merkeze yakın noktalarda 0.5 m) | |

---

## ⚙️ Kontrol Mimarisi

Tüm scriptler aynı **waypoint takip** mimarisini, ROS 2 spin'i ayrı bir thread'de çalıştırarak kullanır (`time.sleep` kontrol döngüsünü engellemesin diye):

```
Odometry (/odom/ground_truth)  →  Konum/Yaw Al  →  Waypoint'e Yönel  →  İleri Git  →  Sonraki Waypoint
   ↑                                                                                          │
   └───────────────────────────────── ROS 2 spin (ayrı thread) ─────────────────────────────┘
                                                                              ↓ Twist (/cmd_vel)
```

- **Yönelim kontrolü:** `don()` fonksiyonu — açısal hataya oransal P kontrolcü (`az = hata × 0.02–0.025`, ±0.7 rad/s ile sınırlı)
- **İleri hareket:** `git()` fonksiyonu — önce hedefe doğru döner, sonra mesafeye oransal ya da sabit hızla ilerler
- **Açı sapması büyükse** (>20–25°) robot önce yeniden döner, ileri gitmeyi keser
- **Tolerans:** Yörüngeye göre 0.2–0.5 m hedef yarıçapı; daire/elips/sonsuzlukta robot waypoint'lerde durmadan akıcı geçiş yapar, karede her köşede kısa bir duraklama (`dur()`) vardır

---

## 🗺️ Yol Haritası

- [x] Kare yörünge
- [x] Daire yörünge
- [x] Elips yörünge
- [x] Sonsuzluk (∞) yörüngesi
- [ ] Üçgen yörünge
- [ ] Spiral yörünge
- [ ] Rviz2 yörünge görselleştirme
- [ ] Launch dosyaları
- [ ] Hız sapma analizi grafiklerinin repoya eklenmesi (MATLAB)

---

## 📄 Lisans

MIT License
