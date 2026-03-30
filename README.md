# 🌕 ROTAY — Ay Yüzeyi İçin Otonom Rota Optimizasyonu

> **NEXUS Team** · TUA Hackathon Projesi  
> *Ebrar Acar · Gülsüm Nur Kolçak · Zehra Betül Türkel · Sevgi Nisa Baysal*

---

## 📌 Genel Bakış

ROTAY, kraterler, taşlar ve pürüzlü arazi gibi tehlikelerle dolu Ay yüzeyinde çalışan roverlar için geliştirilmiş hibrit bir otonom navigasyon sistemidir. Sistem yalnızca *en kısa yolu* bulmak yerine **mesafe**, **güvenlik mesafesi (clearance)** ve **arazi pürüzlülüğünü** dengeleyen çok profilli bir planlama yaklaşımı kullanır.

<img width="665" height="338" alt="Screenshot from 2026-03-29 12-07-45" src="https://github.com/user-attachments/assets/60b40918-436e-491e-9765-7c012c51eb77" />


---

## 🏗️ Sistem Mimarisi

Sistem iki planlama katmanı üzerinde çalışır:

### 🌍 Global Rota Planlama (Çevrimdışı)
- Görev öncesinde uydu/haritalama verileriyle hesaplanır
- Genetik Algoritma (GA) ile çoklu waypoint dizisi optimize edilir
- Her waypoint segmenti clearance ağırlıklı A* ile birleştirilir

### ⚡ Lokal Reaktif Planlama (Gerçek Zamanlı)
- Hareket sırasında beklenmedik engelleri tespit eder
- Engel bölgesini şişirir, clearance ağırlığını artırır
- Global planı bozmadan lokal A* ile yeniden rota hesaplar

<img width="679" height="421" alt="Screenshot from 2026-03-29 14-34-57" src="https://github.com/user-attachments/assets/e34d0efa-c10f-4d23-81c6-88cc8f837b91" />


---

## 🎯 Üç Strateji Profili

| Özellik | 🚀 Hızlı | ⚖️ Dengeli | 🛡️ Güvenli |
|---|---|---|---|
| Mesafe Ağırlığı | Yüksek | Orta | Düşük |
| Clearance Ağırlığı | Düşük | Orta | Yüksek |
| Pürüzlülük Ağırlığı | Düşük | Orta | Orta |
| Rota Uzunluğu | En Kısa | Dengeli | En Uzun |
| Güvenlik Marjı | Minimum | Orta | Maksimum |
| Kullanım Durumu | Düz zemin / acil görev | Standart operasyon | Tehlikeli arazi |

**Maliyet Fonksiyonu:**
```
f(n) = g(n) + h(n)
g(n) = w₁·mesafe + w₂·(1/clearance) + w₃·pürüzlülük
```
w₁, w₂, w₃ seçilen profile göre otomatik ayarlanır.

<img width="498" height="209" alt="Screenshot from 2026-03-29 11-59-35" src="https://github.com/user-attachments/assets/d0075d85-6dca-42ea-a694-fc99e3c27cd1" />


---

## 🗂️ Veri Seti: MoonPlanBench

NASA LRO (Lunar Reconnaissance Orbiter) yükseklik verilerine dayanan **[MoonPlanBench / PlanetaryPathBench](https://github.com/mchancan/PlanetaryPathBench)** veri seti kullanılmaktadır.

- **Format:** `.npy` NumPy binary dizileri (occupancy grid)
- **Çözünürlük:** 100 m/piksel
- **Kapsam:** Farklı Ay enlem bölgeleri
- **Zorluk Seviyeleri:** 20 farklı sahne karmaşıklığı (`MoonPlanBench-20`)

---

## 🛠️ Kurulum

```bash
git clone https://github.com/aithusa13/nexus_rota_optimizasyonu.git
cd nexus_rota_optimizasyonu
pip install numpy scipy scikit-image matplotlib Pillow PyQt6
```

**Veri Seti:**  
[LDEM_875S_5M.npy](https://drive.google.com/drive/folders/15srtIABvwBSbILQESVvAFPHMc3TCzS_R?usp=drive_link) dosyasını indirip şu klasöre koy:
```
Datasets/MoonPlanBench/MoonPlanBench-10/LDEM_875S_5M.npy
```

---

## 🚀 Kullanım

```bash
python rotAY.py
```

- **1 / 2 / 3** → Profil seç (Hızlı / Dengeli / Güvenli)
- **Enter** → Rotayı onayla ve görevi başlat
- **Esc** → Seçimi iptal et

---

<img width="1016" height="842" alt="Screenshot from 2026-03-29 09-45-59" src="https://github.com/user-attachments/assets/7f7979c1-de5e-4a22-9ce8-929d1080535f" />


<img width="1592" height="892" alt="Screenshot from 2026-03-29 09-46-45" src="https://github.com/user-attachments/assets/30fadfbc-70df-44ca-ac77-bec01ae7442e" />


## 📁 Proje Yapısı

```
nexus_rota_optimizasyonu/
├── rotAY.py                 # Ana uygulama
├── LICENSE
└── Datasets/
    └── MoonPlanBench/
        └── MoonPlanBench-10/
            └── LDEM_875S_5M.npy   # Veri seti buraya
```

---

## 📚 Referanslar

1. [MoonPlanBench: A Benchmark for Planetary Surface Path Planning](https://github.com/mchancan/PlanetaryPathBench)
2. https://www.mdpi.com/2072-4292/17/11/1924#Discussion_and_Future_Works
3. https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning
4. https://www.researchgate.net/publication/399135625_Planetary_Terrain_Datasets_and_Benchmarks_for_Rover_Path_Planning
---

<p align="center">NEXUS Team tarafından ☕ ve 🌕 ile yapıldı</p>


////////////////////////////////////////////////////////////////////////////////////////////

# 🌕 ROTAY — Autonomous Route Optimization for the Lunar Surface

> **NEXUS Team** · Hackathon Project  
> *Ebrar Acar · Gülsüm Nur Kolçak · Zehra Betül Türkel · Sevgi Nisa Baysal*

---

## 📌 Overview

ROTAY is a hybrid autonomous navigation system designed for lunar rovers operating in hazardous environments filled with craters, rocks, and rough terrain. Rather than finding just the *shortest* path, the system balances **distance**, **safety clearance**, and **terrain roughness** using a multi-profile planning approach.



---

## 🏗️ System Architecture

The system operates on two planning layers:

### 🌍 Global Route Planning (Offline)
- Pre-mission route computed from satellite/mapping data
- Genetic Algorithm (GA) optimizes a multi-waypoint sequence
- Each waypoint segment connected via clearance-weighted A*

### ⚡ Local Reactive Planning (Real-time)
- Detects unexpected obstacles during movement
- Inflates obstacle region and increases clearance weight
- Re-routes locally using A* without disrupting the global plan

---

## 🎯 Three Strategy Profiles

| Feature | 🚀 Fast | ⚖️ Balanced | 🛡️ Safe |
|---|---|---|---|
| Distance Weight | High | Medium | Low |
| Clearance Weight | Low | Medium | High |
| Roughness Weight | Low | Medium | Medium |
| Path Length | Shortest | Moderate | Longest |
| Safety Margin | Minimum | Medium | Maximum |
| Best For | Flat terrain / emergency | Standard ops | Hazardous terrain |

**Cost Function:**
```
f(n) = g(n) + h(n)
g(n) = w₁·distance + w₂·(1/clearance) + w₃·roughness
```
w₁, w₂, w₃ are automatically set based on the selected profile.


---

## 🗂️ Dataset: MoonPlanBench

We use the **[MoonPlanBench / PlanetaryPathBench](https://github.com/mchancan/PlanetaryPathBench)** dataset, built on NASA LRO (Lunar Reconnaissance Orbiter) elevation data.

- **Format:** `.npy` NumPy binary arrays (occupancy grids)
- **Resolution:** 100 m/pixel
- **Coverage:** Multiple lunar latitude regions
- **Difficulty Levels:** 20 scene complexity variants (`MoonPlanBench-20`)

---

## 🛠️ Installation

```bash
git clone https://github.com/aithusa13/nexus_rota_optimizasyonu.git
cd nexus_rota_optimizasyonu
pip install numpy scipy scikit-image matplotlib Pillow PyQt6
```

**Dataset:**  
Download [LDEM_875S_5M.npy](https://drive.google.com/drive/folders/15srtIABvwBSbILQESVvAFPHMc3TCzS_R?usp=drive_link) and place it here:
```
Datasets/MoonPlanBench/MoonPlanBench-10/LDEM_875S_5M.npy
```

---

## 🚀 Usage

```bash
python rotAY.py
```

- **1 / 2 / 3** → Select profile (Fast / Balanced / Safe)
- **Enter** → Confirm route and start mission
- **Esc** → Cancel selection

---

## 📁 Project Structure

```
nexus_rota_optimizasyonu/
├── rotAY.py                 # Main application
├── LICENSE
└── Datasets/
    └── MoonPlanBench/
        └── MoonPlanBench-10/
            └── LDEM_875S_5M.npy   # Place dataset here
```

---

## 📚 References

1. [MoonPlanBench: A Benchmark for Planetary Surface Path Planning](https://github.com/mchancan/PlanetaryPathBench)

---

<p align="center">Made with ☕ and 🌕 by NEXUS Team</p>
