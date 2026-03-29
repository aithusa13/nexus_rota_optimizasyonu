

from __future__ import annotations

import heapq, random, time, threading
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict

import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
plt.rcParams['figure.raise_window'] = True
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy.ndimage import distance_transform_edt
from skimage.transform import resize

RANDOM_SEED = 42
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

Point = Tuple[int, int]
Path  = List[Point]

@dataclass
class TerrainFeatures:
    grid:      np.ndarray
    clearance: np.ndarray
    raw:       np.ndarray
    shape:     Tuple[int, int]


def load_terrain(path: str,
                 target_size: Tuple[int,int] = (100,100)) -> TerrainFeatures:
    raw  = np.load(path)
    grid = resize(raw, target_size, order=0,
                  anti_aliasing=False,
                  preserve_range=True).astype(np.uint8)
    cl   = distance_transform_edt((grid == 0).astype(float))
    return TerrainFeatures(grid=grid, clearance=cl,
                           raw=grid.copy(), shape=grid.shape)


def safe_point(grid: np.ndarray, preferred: Point) -> Point:
    if grid[preferred] == 0:
        return preferred
    H, W = grid.shape
    for r in range(1, 20):
        for dr in range(-r, r+1):
            for dc in range(-r, r+1):
                nr, nc = preferred[0]+dr, preferred[1]+dc
                if 0<=nr<H and 0<=nc<W and grid[nr,nc] == 0:
                    return (nr, nc)
    raise RuntimeError("Boş hücre bulunamadı")



_DIR8 = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def astar(grid: np.ndarray,
          clearance: np.ndarray,
          start: Point,
          goal: Point,
          w_danger: float = 0.5,
          blocked: Optional[List[Point]] = None) -> Optional[Path]:

    H, W     = grid.shape
    blk_set  = set(blocked) if blocked else set()
    g:  Dict[Point, float] = {start: 0.0}
    cf: Dict[Point, Point] = {}
    heap = [(np.hypot(goal[0]-start[0], goal[1]-start[1]), start)]

    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            path: Path = []
            while cur in cf:
                path.append(cur); cur = cf[cur]
            path.append(start)
            return path[::-1]
        for dr, dc in _DIR8:
            nb = (cur[0]+dr, cur[1]+dc)
            if not (0<=nb[0]<H and 0<=nb[1]<W): continue
            if grid[nb] or nb in blk_set: continue
            pen  = w_danger / (clearance[nb] + 1.0)
            cost = np.hypot(dr, dc) * (1.0 + pen)
            tg   = g[cur] + cost
            if tg < g.get(nb, 1e18):
                cf[nb] = cur; g[nb] = tg
                heapq.heappush(heap,
                    (tg + np.hypot(goal[0]-nb[0], goal[1]-nb[1]), nb))
    return None

def lokal_replan(feat: TerrainFeatures,
                 cur_pos: Point,
                 goal: Point,
                 surpriz: Point,
                 inflate_r: int = 4) -> Optional[Path]:

    H, W    = feat.shape
    blocked = []
    for dr in range(-inflate_r, inflate_r+1):
        for dc in range(-inflate_r, inflate_r+1):
            if dr**2 + dc**2 <= inflate_r**2:
                bp = (surpriz[0]+dr, surpriz[1]+dc)
                if 0<=bp[0]<H and 0<=bp[1]<W:
                    blocked.append(bp)

    return astar(feat.grid, feat.clearance,
                 cur_pos, goal,
                 w_danger=1.2,   
                 blocked=blocked)


def build_path(waypoints: List[Point],
               feat: TerrainFeatures,
               w_danger: float = 0.5) -> Optional[Path]:
    full: Path = []
    for i in range(len(waypoints)-1):
        seg = astar(feat.grid, feat.clearance,
                    waypoints[i], waypoints[i+1], w_danger)
        if seg is None: return None
        full.extend(seg[:-1])
    full.append(waypoints[-1])
    return full


@dataclass
class Weights:
    distance:   float = 0.4
    clearance:  float = 0.3
    smoothness: float = 0.3

@dataclass
class PathMetrics:
    total_distance: float
    avg_clearance:  float
    smoothness:     float
    fitness:        float

def compute_metrics(path: Path,
                    feat: TerrainFeatures,
                    w: Weights) -> PathMetrics:
    arr   = np.array(path, dtype=float)
    diffs = np.diff(arr, axis=0)
    dist  = float(np.sum(np.hypot(diffs[:,0], diffs[:,1])))

    smooth = 0.0
    if len(diffs) > 1:
        norms = np.linalg.norm(diffs, axis=1, keepdims=True) + 1e-9
        unit  = diffs / norms
        dots  = np.clip(np.sum(unit[:-1]*unit[1:], axis=1), -1, 1)
        smooth = float(np.mean(np.degrees(np.arccos(dots))))

    rows, cols = zip(*path)
    cl  = feat.clearance[rows, cols]
    mxd = np.hypot(*feat.shape)
    fit = (w.distance   * dist / mxd +
           w.clearance  * float(np.mean(1.0/(cl+0.1))) / (1.0/0.1) +
           w.smoothness * smooth / 180.0)

    return PathMetrics(dist, float(np.mean(cl)), smooth, fit)

@dataclass
class Individual:
    waypoints: List[Point]
    path:    Optional[Path]        = field(default=None, repr=False)
    metrics: Optional[PathMetrics] = field(default=None, repr=False)

    @property
    def fitness(self) -> float:
        return self.metrics.fitness if self.metrics else 1e9

def _free(grid: np.ndarray) -> Point:
    free = np.argwhere(grid == 0)
    idx  = random.randint(0, len(free)-1)
    return (int(free[idx][0]), int(free[idx][1]))

def run_ga(feat: TerrainFeatures,
           start: Point, goal: Point,
           w: Weights,
           n_wp: int = 3,
           pop_size: int = 20,
           n_gen:    int = 20,
           elite_frac: float = 0.15,
           base_mut:   float = 0.25,
           ) -> Tuple[Optional[Path], Optional[PathMetrics], List[float]]:

    wd      = w.clearance
    n_elite = max(1, int(pop_size * elite_frac))

    def _eval(ind: Individual) -> Individual:
        ind.path = build_path(ind.waypoints, feat, wd)
        if ind.path:
            ind.metrics = compute_metrics(ind.path, feat, w)
        return ind

    pop = [_eval(Individual(
               [start] + [_free(feat.grid) for _ in range(n_wp)] + [goal]))
           for _ in range(pop_size)]

    best_path, best_met, best_fit = None, None, 1e9
    history: List[float] = []

    for gen in range(n_gen):
        pop.sort(key=lambda x: x.fitness)
        if pop[0].fitness < best_fit:
            best_fit  = pop[0].fitness
            best_path = pop[0].path
            best_met  = pop[0].metrics
        history.append(best_fit)

        mut = base_mut * (1.0 - 0.5*gen/n_gen)
        nxt = pop[:n_elite]
        while len(nxt) < pop_size:
            p1 = min(random.sample(pop, 3), key=lambda x: x.fitness)
            p2 = min(random.sample(pop, 3), key=lambda x: x.fitness)
            i1, i2 = p1.waypoints[1:-1], p2.waypoints[1:-1]
            if i1 and i2:
                cut  = random.randint(1, len(i1))
                cwps = [start] + i1[:cut] + i2[cut:] + [goal]
            else:
                cwps = p1.waypoints[:]
            child = Individual([
                wp if random.random() > mut else _free(feat.grid)
                for wp in cwps])
            child.waypoints[0]  = start
            child.waypoints[-1] = goal
            nxt.append(_eval(child))
        pop = nxt

    return best_path, best_met, history


SCENARIOS: Dict[str, Weights] = {
    "Hızlı (Fast)":   Weights(distance=0.8, clearance=0.1, smoothness=0.1),
    "Güvenli (Safe)": Weights(distance=0.1, clearance=0.7, smoothness=0.2),
    "Dengeli (Eco)":  Weights(distance=0.4, clearance=0.3, smoothness=0.3),
}
COLORS = {
    "Hızlı (Fast)":   "#FF4136",
    "Güvenli (Safe)": "#00CFFF",
    "Dengeli (Eco)":  "#2ECC40",
}

def run_and_visualize(feat: TerrainFeatures,
                      start: Point, goal: Point,
                      pop_size: int = 20,
                      n_gen: int    = 20, 
                      data_path: str = "") -> None:

    ga_results: Dict = {}          
    ga_done   = threading.Event()  
    ui_state  = {
        "phase":    "computing",   
        "selected": None,
        "onayla_locked": False,
    }


    def ga_worker():
        print("GA hesaplanıyor...")
        for name, w in SCENARIOS.items():
            t0   = time.perf_counter()
            path, met, hist = run_ga(feat, start, goal, w,
                                     pop_size=pop_size, n_gen=n_gen)
            ga_results[name] = (path, met, hist)
            if met:
                print(f"  {name:<18} dist={met.total_distance:.1f} "
                      f"cl={met.avg_clearance:.2f} "
                      f"({time.perf_counter()-t0:.1f}s)")
        ga_done.set()
        print("GA tamamlandı — rotalar çiziliyor...")

    t = threading.Thread(target=ga_worker, daemon=True)
    t.start()


    fig = plt.figure(figsize=(16, 9), facecolor="#0d0d0d")
    try:
        fig.canvas.manager.set_window_title("Türkiye Ay Rover'ı v7")
    except Exception:
        pass
    gs  = gridspec.GridSpec(3, 3, figure=fig,
                            left=0.05, right=0.95,
                            top=0.92,  bottom=0.08,
                            wspace=0.35, hspace=0.45)

    ax_map  = fig.add_subplot(gs[:, 0:2])
    ax_conv = fig.add_subplot(gs[0, 2])
    ax_met  = fig.add_subplot(gs[1, 2])
    ax_info = fig.add_subplot(gs[2, 2])
    for ax in [ax_map, ax_conv, ax_met, ax_info]:
        ax.set_facecolor("#111111")

    import os
    from PIL import Image as PILImage
    _data_path = data_path
    _jpeg_path = _data_path.replace(
        'Datasets/MoonPlanBench/MoonPlanBench-10/', 
        'Datasets/MoonPlanBench/Background/jpeg/'
    ).replace('.npy', '.jpeg')
    if not os.path.exists(_jpeg_path):
        _jpeg_path = _data_path.replace('.npy', '.jpeg')
    if not os.path.exists(_jpeg_path):
        _jpeg_path = 'LDEM_875S_5M.jpeg'

    if os.path.exists(_jpeg_path):
        _bg = np.array(PILImage.open(_jpeg_path))
        ax_map.imshow(_bg, origin='upper', alpha=1.0,
                      extent=[0, feat.shape[1], feat.shape[0], 0],
                      aspect='auto')
        ax_map.imshow(feat.clearance, cmap='hot', origin='upper',
                      alpha=0.18,
                      extent=[0, feat.shape[1], feat.shape[0], 0],
                      aspect='auto')
    else:
        ax_map.imshow(feat.clearance, cmap='inferno',
                      origin='upper', alpha=0.85)


    obs_rgba = np.zeros((*feat.grid.shape, 4), dtype=np.float32)
    obs_rgba[feat.grid == 1] = [0.05, 0.05, 0.05, 0.7]
    ax_map.imshow(obs_rgba, origin='upper',
                  extent=[0, feat.shape[1], feat.shape[0], 0],
                  aspect='auto')
    ax_map.scatter([start[1], goal[1]], [start[0], goal[0]],
                   c=["#00FF00","#FF0000"], s=150, zorder=7,
                   edgecolors="white", linewidths=1.5)
    ax_map.annotate("START", (start[1],start[0]),
                    xytext=(start[1]+2, start[0]-5),
                    color="#00FF00", fontsize=8)
    ax_map.annotate("GOAL",  (goal[1], goal[0]),
                    xytext=(goal[1]+2,  goal[0]-5),
                    color="#FF0000", fontsize=8)
    ax_map.tick_params(colors="gray")

    wait_txt = ax_map.text(
        0.5, 0.5, "GA hesaplanıyor...\nlütfen bekleyin",
        transform=ax_map.transAxes, ha="center", va="center",
        color="white", fontsize=14, fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.5",
                  facecolor="#1a1a1a", alpha=0.85))

    status_txt = ax_map.text(
        0.5, -0.04,
        "Rotalar hesaplanıyor, lütfen bekleyin...",
        transform=ax_map.transAxes, ha="center", va="top",
        color="#888888", fontsize=8,
        bbox=dict(boxstyle="round,pad=0.3",
                  facecolor="#1a1a1a", edgecolor="#444", alpha=0.9))

    ax_info.axis("off")
    btn_onayla = Button(
        fig.add_axes([0.685, 0.10, 0.27, 0.04], facecolor="#1a1a1a"),
        "Hesaplanıyor...", color="#1a1a1a", hovercolor="#1a1a1a")
    btn_onayla.label.set_color("#555555")
    btn_onayla.label.set_fontsize(10)
    btn_onayla.label.set_fontweight("bold")

    fig.suptitle(
        "Türkiye Ay Rover'ı — Global: A*+Genetic Algoritma + Lokal: A*",
        color="white", fontsize=12, fontweight="bold", y=0.97)
    fig.text(0.5, 0.002,
             "1=Hızlı  2=Güvenli  3=Dengeli  |  Enter=Onayla  |  Esc=İptal",
             ha="center", va="bottom", color="#555555", fontsize=7.5)

    line_objects: Dict = {}
    draw_q: List  = []   
    draw_s = {"si": 0, "di": 0}  
    STEP   = 4

    def poll_ga(_frame):
        if ui_state["phase"] != "computing": return
        if not ga_done.is_set(): return

        ui_state["phase"] = "drawing"
        wait_txt.set_visible(False)

        ax_conv.set_title("GA Yakınsama", color="white", fontsize=10)
        ax_conv.set_xlabel("Nesil", color="gray", fontsize=8)
        ax_conv.set_ylabel("Fitness ↓", color="gray", fontsize=8)
        ax_conv.tick_params(colors="gray", labelsize=7)
        for name in SCENARIOS:
            if name in ga_results:
                _, _, hist = ga_results[name]
                ax_conv.plot(hist, color=COLORS[name],
                             linewidth=1.8, label=name)
        ax_conv.legend(facecolor="#1a1a1a", edgecolor="#444",
                       labelcolor="white", fontsize=7)

        ax_met.axis("off")
        ax_met.set_title("Karşılaştırma", color="white",
                          fontsize=10, pad=4)
        cols = ["Strateji","Mesafe","Clearance","Pürüz"]
        td, tc = [], []
        for name in SCENARIOS:
            if name not in ga_results: continue
            _, m, _ = ga_results[name]
            row = ([name, f"{m.total_distance:.1f}",
                    f"{m.avg_clearance:.2f}", f"{m.smoothness:.1f}°"]
                   if m else [name,"—","—","—"])
            td.append(row); tc.append(["#1a1a1a"]*4)
        tbl = ax_met.table(cellText=td, colLabels=cols,
                            cellLoc="center", loc="center",
                            cellColours=tc)
        tbl.auto_set_font_size(False); tbl.set_fontsize(8)
        for (r,c), cell in tbl.get_celld().items():
            cell.set_edgecolor("#333")
            if r == 0:
                cell.set_facecolor("#222")
                cell.set_text_props(color="#FFD700", fontweight="bold")
            elif r-1 < len(td):
                cell.set_text_props(
                    color=COLORS.get(td[r-1][0], "white"))

        for name in SCENARIOS:
            if name not in ga_results: continue
            path = ga_results[name][0]
            if not path: continue
            draw_q.append((name, path))
            ln, = ax_map.plot([], [], color=COLORS[name],
                               linewidth=2.5, alpha=0.9, label=name)
            ln.set_picker(8)
            line_objects[name] = ln

        ax_map.legend(facecolor="#1a1a1a", edgecolor="#444",
                      labelcolor="white", fontsize=9)
        ax_map.set_title(
            "Clearance haritası | Rota seçimi",
            color="white", fontsize=10, pad=8)
        status_txt.set_text(
            "Rotalar çiziliyor...")
        status_txt.set_color("#AAAAAA")

    def anim_frame(frame):
        if ui_state["phase"] == "computing":
            poll_ga(frame)
            return

        if ui_state["phase"] == "drawing":
            si = draw_s["si"]
            if si >= len(draw_q):
                ui_state["phase"] = "idle"
                status_txt.set_text(
                    "Rota seçin: çizgiye tıklayın  |  "
                    "1=Hızlı  2=Güvenli  3=Dengeli  |  Enter=Onayla")
                status_txt.set_color("#AAAAAA")
                btn_onayla.label.set_text("Rota Seçin →")
                btn_onayla.label.set_color("#666666")
                return

            name, path = draw_q[si]
            di  = draw_s["di"]
            end = min(di + STEP, len(path))
            py, px = zip(*path[:end])
            line_objects[name].set_data(px, py)
            draw_s["di"] = end
            if end >= len(path):
                draw_s["si"] += 1
                draw_s["di"]  = 0

    main_ani = FuncAnimation(fig, anim_frame,
                              interval=30, blit=False, repeat=False,
                              frames=100000)
    fig._main_ani = main_ani

    def highlight(name: str) -> None:
        for n, ln in line_objects.items():
            ln.set_linewidth(4.5 if n == name else 1.2)
            ln.set_alpha(1.0   if n == name else 0.2)
            ln.set_zorder(10   if n == name else 5)

    def on_pick(event) -> None:
        if ui_state["phase"] not in ("idle",): return
        for name, ln in line_objects.items():
            if event.artist is ln:
                ui_state["selected"] = name
                highlight(name)
                m = ga_results[name][1]
                if m:
                    status_txt.set_text(
                        f"SEÇİLİ: {name}  |  "
                        f"Mesafe:{m.total_distance:.1f}  "
                        f"Cl:{m.avg_clearance:.2f}  "
                        f"Pürüz:{m.smoothness:.1f}°  |  Enter=Onayla")
                    status_txt.set_color(COLORS[name])
                btn_onayla.label.set_text("✓  Rotayı Onayla  (Enter)")
                btn_onayla.label.set_color("#00FF88")
                btn_onayla.ax.set_facecolor("#1a4a2a")
                fig.canvas.draw_idle()
                print(f"\n[Seçildi] {name}")
                break

    fig.canvas.mpl_connect("pick_event", on_pick)

    rota_listesi = list(SCENARIOS.keys())
    def on_key(event) -> None:
        if event.key in ("1","2","3"):
            idx = int(event.key)-1
            if idx < len(rota_listesi):
                class FP:
                    artist = line_objects.get(rota_listesi[idx])
                if FP.artist: on_pick(FP())
        elif event.key == "enter":
            on_onayla(None)
        elif event.key == "escape":
            ui_state["selected"] = None
            for ln in line_objects.values():
                ln.set_linewidth(2.5); ln.set_alpha(0.9)
            status_txt.set_text(
                "Rota seçin: tıkla veya 1/2/3")
            status_txt.set_color("#888888")
            btn_onayla.label.set_text("Rota Seçin →")
            btn_onayla.label.set_color("#666666")
            btn_onayla.ax.set_facecolor("#1a1a1a")
            fig.canvas.draw_idle()
    fig.canvas.mpl_connect("key_press_event", on_key)


    def on_onayla(_) -> None:
        if ui_state["onayla_locked"]: return
        if ui_state["phase"] != "idle": return
        if ui_state["selected"] is None:
            print("[!] Önce rota seçin.")
            return

        ui_state["onayla_locked"] = True
        ui_state["phase"]         = "running"

        name  = ui_state["selected"]
        path  = ga_results[name][0]
        renk  = COLORS[name]
        met   = ga_results[name][1]

        print("\n" + "═"*55)
        print("   ROVER ROTASI ONAYLANDI")
        print("═"*55)
        print(f"   Strateji  : {name}")
        if met:
            print(f"   Mesafe    : {met.total_distance:.1f} px")
            print(f"   Clearance : {met.avg_clearance:.2f}")
        print("═"*55)

        for n, ln in line_objects.items():
            if n != name: ln.set_data([], [])

        btn_onayla.label.set_text("Görev Başladı...")
        btn_onayla.label.set_color("#FFAA00")
        btn_onayla.ax.set_facecolor("#2a2a00")
        fig.canvas.draw_idle()

        rover_dot, = ax_map.plot([], [], 'o', color='white', markeredgecolor=renk,
                                  markeredgewidth=2.5, markersize=11, zorder=20)
        trail_ln, = ax_map.plot([], [], '-', color=renk, linewidth=1.5, alpha=0.6, zorder=19)
        adim_txt  = ax_map.text(0.02, 0.97, "", transform=ax_map.transAxes,
                                color="white", fontsize=8, va="top",
                                bbox=dict(boxstyle="round,pad=0.2", facecolor="#000", alpha=0.65))

        surpriz_dot, = ax_map.plot([], [], 'X', color='#FF0000', markersize=14, zorder=25,
                                    markeredgecolor='white', markeredgewidth=1.5)

        rv = {
            "idx":       0,
            "path":      list(path), 
            "surpriz":   False,
            "replanned": False,
        }

        def randomize_obstacle():
            total_pts = len(rv["path"])
            target_idx = int(total_pts * 0.55)
            base_r, base_c = rv["path"][target_idx]
            for _ in range(30):
                off_r = random.randint(-6, 6)
                off_c = random.randint(-6, 6)
                nr, nc = max(0, min(feat.shape[0]-1, base_r + off_r)), max(0, min(feat.shape[1]-1, base_c + off_c))
                if feat.grid[nr, nc] == 0:
                    return (nr, nc)
            return (base_r, base_c)

        surpriz_pos = randomize_obstacle()
        total_pts = len(rv["path"])
        interval_ms = max(25, min(70, 5000 // total_pts))

        def rover_frame(frame):
            i     = rv["idx"]
            cpath = rv["path"]
            tot   = len(cpath)

            if i >= tot:
                ui_state["phase"] = "idle"
                ui_state["selected"] = None
                ui_state["onayla_locked"] = False
                
                rover_dot.remove()
                trail_ln.set_data([], [])
                surpriz_dot.set_data([], [])
                adim_txt.set_text("")
                
                for n, ln in line_objects.items():
                    p_orig = ga_results[n][0]
                    py_o, px_o = zip(*p_orig)
                    ln.set_data(px_o, py_o)
                    ln.set_linewidth(2.5)
                    ln.set_alpha(0.9)
                
                btn_onayla.label.set_text("Rota Seçin →")
                btn_onayla.label.set_color("#666666")
                btn_onayla.ax.set_facecolor("#1a1a1a")
                ax_map.set_title("Görev Tamamlandı. Yeni Strateji Seçebilirsiniz.", color="#00FF88")
                status_txt.set_text("Rota seçin: tıkla veya 1/2/3")
                status_txt.set_color("#888888")
                
                fig.canvas.draw_idle()
                if hasattr(fig, '_rover_ani'):
                    fig._rover_ani.event_source.stop()
                return

            r, c = cpath[i]

            if not rv["surpriz"] and i >= int(len(path)*0.45):
                rv["surpriz"] = True
                surpriz_dot.set_data([surpriz_pos[1]], [surpriz_pos[0]])
                ax_map.set_title("⚠ Sürpriz Engel! Lokal A* devreye giriyor...", color="#FF4444")
                status_txt.set_text("⚠ Sürpriz Engel Algılandı!")
                status_txt.set_color("#FF4444")

            if not rv["replanned"] and rv["surpriz"] and i >= int(len(path)*0.50):
                rv["replanned"] = True
                new_path = lokal_replan(feat, (r, c), cpath[-1], surpriz_pos)
                if new_path:
                    rv["path"] = cpath[:i] + new_path
                    py, px = zip(*rv["path"])
                    line_objects[name].set_data(px, py)
                    ax_map.set_title("Lokal A* — Yeni rota hesaplandı", color="#FFAA00")

            rover_dot.set_data([c], [r])
            if i % 2 == 0:
                sub = rv["path"][:i+1]
                tr, tc = [p[0] for p in sub], [p[1] for p in sub]
                trail_ln.set_data(tc, tr)
            
            adim_txt.set_text(f"Adım: {i+1}/{len(rv['path'])}")
            rv["idx"] += 1
            return rover_dot, trail_ln, adim_txt

        rover_ani = FuncAnimation(fig, rover_frame, frames=100000,
                                   interval=interval_ms, blit=False, repeat=False)
        fig._rover_ani = rover_ani

    btn_onayla.on_clicked(on_onayla)
    plt.show()

    


# ══════════════════════════════════════════════════════════════════
#  7. GİRİŞ NOKTASI
# ══════════════════════════════════════════════════════════════════
def main() -> None:
    DATA_PATH = "Datasets/MoonPlanBench/MoonPlanBench-10/LDEM_875S_5M.npy"
    feat  = load_terrain(DATA_PATH, target_size=(100, 100))
    start = safe_point(feat.grid, (10, 10))
    goal  = safe_point(feat.grid, (90, 90))

    print(f"Başlangıç  : {start}  →  Hedef: {goal}")
    print(f"Engel oranı: {feat.grid.mean()*100:.1f}%")
    print(f"Harita     : {feat.grid.shape}")
    run_and_visualize(feat, start, goal, pop_size=20, n_gen=20, data_path=DATA_PATH)


if __name__ == "__main__":
    main()