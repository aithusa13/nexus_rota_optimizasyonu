# Ay Rover Rota Planlama — NEXUS Team

Türkiye'nin Ay rover'ı için geliştirilmiş hibrit global + lokal yol planlama sistemi. NASA'nın gerçek Ay yüzeyi verileri (MoonPlanBench) üzerinde çalışır.

---

## Nasıl Çalışır?

**Global:** Genetik Algoritma waypoint dizisi optimize eder, her segment clearance ağırlıklı A* ile birleştirilir.  
**Lokal:** Görev sırasında sürpriz engel çıktığında A* anlık olarak devreye girer ve rotayı günceller.

Üç misyon profili sunulur:

| Profil | Mesafe | Clearance | Pürüzlülük |
|---|---|---|---|
| 🔴 Hızlı (Fast) | 0.8 | 0.1 | 0.1 |
| 🟢 Dengeli (Eco) | 0.4 | 0.3 | 0.3 |
| 🔵 Güvenli (Safe) | 0.1 | 0.7 | 0.2 |

---

## Kurulum

```bash
pip install numpy scipy scikit-image matplotlib Pillow PyQt6
```

---

## Kullanım

```bash
python rover.py
```

Veri seti: [MoonPlanBench — NASA LOLA DEM](https://drive.google.com/drive/folders/15srtIABvwBSbILQESVvAFPHMc3TCzS_R?usp=drive_link)  
`LDEM_875S_5M.npy` dosyasını `Datasets/MoonPlanBench/MoonPlanBench-10/` klasörüne koy.

---

## Arayüz

- **1 / 2 / 3** → Profil seç
- **Enter** → Rotayı onayla ve görevi başlat
- **Esc** → Seçimi iptal et

---

## Teknik Detaylar

| | |
|---|---|
| Dil | Python 3.10 |
| GA | Pop:20, Nesil:20, Mutasyon:0.25→0.12, Elit:%15 |
| A* | 8-bağlantılı, clearance penaltı: `w/(cl+1)` |
| Lokal replanning | `inflate_r=4`, `w_danger=1.2` |
| Harita | 100×100 occupancy grid |

---

*TUA Astro Hackathon 2025 | NEXUS Team*
