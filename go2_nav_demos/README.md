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
│   └── kare_world.world      # Gazebo 5×5 m kare alan (sarı duvarlar)
├── scripts/
│   ├── kare_tur.py           # ✅ Kare yörünge (tamamlandı)
│   ├── elips_tur.py          # 🔄 Elips yörünge (tamamlandı)
│   └── sonsuzluk_tur.py      # ♾️  Sonsuzluk (∞ / lemniskat) yörüngesi (tamamlandı)
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
# Gazebo dünyasını aç
ros2 launch gazebo_ros gazebo.launch.py \
  world:=$(pwd)/worlds/kare_world.world

# Robotu spawn et (ayrı terminal)
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity go2
```

### Scriptleri Çalıştır

```bash
# Kare tur
python3 scripts/kare_tur.py

# Elips tur
python3 scripts/elips_tur.py

# Sonsuzluk (∞) turu
python3 scripts/sonsuzluk_tur.py
```

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
| Hız | 0.4 m/s |

---

### 〇 Elips Turu (`elips_tur.py`)

Robot, parametrik elips yörüngesini 36 waypoint ile izler.

```
x(t) = a·cos(t)
y(t) = b·sin(t)        t ∈ [0, 2π)
```

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `a` | 2.0 m | X yarıçapı |
| `b` | 1.2 m | Y yarıçapı |
| `N` | 36 | Waypoint sayısı |
| Tur sayısı | 3 | |
| Hız | 0.35 m/s | |

---

### ♾️ Sonsuzluk Turu (`sonsuzluk_tur.py`)

Robot, **Bernoulli Lemniskatı** yörüngesini izler — ortadan geçen sekiz şekli.

```
x(t) = a·cos(t) / (1+sin²(t))
y(t) = a·sin(t)·cos(t) / (1+sin²(t))       t ∈ [0, 2π)
```

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `a` | 1.8 m | Ölçek katsayısı |
| `N` | 72 | Waypoint sayısı |
| Tur sayısı | 3 | |
| Hız | 0.30 m/s | |

---

## ⚙️ Kontrol Mimarisi

Tüm scriptler aynı basit **waypoint takip** mimarisini kullanır:

```
Odometry  →  Konum/Yaw Al  →  Waypoint'e Yönel  →  İleri Git  →  Sonraki Waypoint
   ↑                                                                      │
   └──────────────────────────── ROS 2 spin ────────────────────────────┘
```

- **Yönelim kontrolü:** Oransal P kontrolcü (`az = hata × 0.025`)
- **İleri hız:** Mesafeye oransal, maksimum hızla sınırlı
- **Tolerans:** 0.15–0.20 m hedef yarıçapı

---

## 🗺️ Yol Haritası

- [x] Kare yörünge
- [x] Elips yörünge  
- [x] Sonsuzluk (∞) yörüngesi
- [ ] Üçgen yörünge
- [ ] Spiral yörünge
- [ ] Rviz2 yörünge görselleştirme
- [ ] Launch dosyaları

---

## 📄 Lisans

MIT License
