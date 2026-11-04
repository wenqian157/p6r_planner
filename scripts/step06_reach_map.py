"""Step 6: reachable-position maps of the UR12e at discrete base positions (lecture-7 slide 22 style).

Fix the flange orientation and height, lay a grid of flange positions (x_w, y_w) over the floor plane,
run the analytical IK at every grid point and colour the point by the number of branches found (0..8).
Joint limits are the project convention +/- pi, so the count is purely geometric.
One panel per base position x in X_BASES; the base and its UMR footprint are drawn in each panel.

Run from the repo root:   python scripts/step06_reach_map.py
Output: results/figures/reach_map_x0.png (x = 0 alone), results/figures/reach_maps_x.png (all X_BASES),
        results/step06_reach_map.md
"""
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e import ik_world
from umr_params import UMR_LENGTH, UMR_WIDTH


class Tee:
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
pi = np.pi

# ------------------------------------------------ definition of the maps
X_BASES = [-1.0, 0.0, 1.0]                     # base positions on the rail [m]
Z_FLANGE = 0.9                                 # flange height above the floor [m]
R_FLANGE = np.array([[0, 0, 1],                # flange orientation, columns = flange axes in the world:
                     [0, 1, 0],                #   x_f = -z_w  (URDF flange x = outward tool normal = axis 6, pointing down)
                     [-1, 0, 0]], float)       #   y_f = +y_w,  z_f = +x_w
STEP = 0.02
XS = np.arange(-2.4, 2.4 + 1e-9, STEP)         # grid of flange positions in the world frame [m]
YS = np.arange(-1.4, 1.4 + 1e-9, STEP)
# ---------------------------------------------------------------------------


def count_map(x_base, z, R, xs, ys):
    """Number of IK branches at every grid point (rows = y, cols = x), joint limits +/- pi."""
    C = np.zeros((len(ys), len(xs)), int)
    T = np.eye(4)
    T[:3, :3] = R
    for j, yw in enumerate(ys):
        for i, xw in enumerate(xs):
            T[:3, 3] = [xw, yw, z]
            C[j, i] = len(ik_world(T, x_base))
    return C


# ordinal ramp: 0 = neutral grey (unreachable), 1..8 = one blue hue, light -> dark
COLORS = ["#e8e7e3", "#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#5598e7", "#2a78d6", "#184f95", "#0d366b"]
CMAP = ListedColormap(COLORS)
NORM = BoundaryNorm(np.arange(-0.5, 9.5, 1), CMAP.N)


def save(fig, path, tries=5):
    """savefig with retries: a viewer holding the PNG open makes Windows refuse the write for a moment."""
    for k in range(tries):
        try:
            fig.savefig(path, dpi=200)
            return
        except OSError:
            time.sleep(2)
    fig.savefig(str(path).replace(".png", "_new.png"), dpi=200)


def draw(ax, C, x_base):
    extent = [XS[0] - STEP / 2, XS[-1] + STEP / 2, YS[0] - STEP / 2, YS[-1] + STEP / 2]
    im = ax.imshow(C, origin="lower", extent=extent, cmap=CMAP, norm=NORM, interpolation="nearest")
    ax.add_patch(Rectangle((x_base - UMR_LENGTH / 2, -UMR_WIDTH / 2), UMR_LENGTH, UMR_WIDTH,
                           fill=False, ec="#333", lw=1.2, ls="--"))
    ax.plot(x_base, 0, "k+", ms=10, mew=1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("flange x_w [m]")
    ax.set_ylabel("flange y_w [m]")
    ax.set_title(f"base at x = {x_base:+.1f} m", fontsize=10)
    return im


t0 = time.time()
maps = {x: count_map(x, Z_FLANGE, R_FLANGE, XS, YS) for x in X_BASES}
t_ik = time.time() - t0
fig_dir = HERE.parent / "results" / "figures"
fig_dir.mkdir(parents=True, exist_ok=True)

# single map at x = 0
fig, ax = plt.subplots(figsize=(9, 5.6), constrained_layout=True)
im = draw(ax, maps[0.0], 0.0)
cb = fig.colorbar(im, ax=ax, ticks=range(9), shrink=0.9)
cb.set_label("number of IK branches")
fig.suptitle(f"Reachable-position map, flange z = {Z_FLANGE} m, tool axis pointing down, joint limits +/- pi "
             f"(dashed: UMR footprint, +: axis 1)", fontsize=10)
save(fig, fig_dir / "reach_map_x0.png")

# one row, all base positions
fig, axes = plt.subplots(len(X_BASES), 1, figsize=(9, 3.3 * len(X_BASES)), constrained_layout=True, sharex=True)
for ax, x in zip(axes, X_BASES):
    im = draw(ax, maps[x], x)
cb = fig.colorbar(im, ax=axes, ticks=range(9), shrink=0.6)
cb.set_label("number of IK branches")
fig.suptitle(f"Reachable-position maps for three base positions, flange z = {Z_FLANGE} m, tool axis pointing down, "
             f"joint limits +/- pi", fontsize=10)
save(fig, fig_dir / "reach_maps_x.png")

# ------------------------------------------------ report
print("=== reachable-position maps, joint limits +/- pi (geometric count) ===")
print(f"flange orientation R (world), columns = flange axes:\n{R_FLANGE}")
print("  -> flange outward normal (URDF flange x = DH z6, the tool axis) pointing down; O5 is d6 straight above the flange")
print(f"flange height z = {Z_FLANGE} m  (arm base at 0.42 m, shoulder axis at 0.6007 m)")
print(f"grid: x_w in [{XS[0]:.1f}, {XS[-1]:.1f}] m, y_w in [{YS[0]:.1f}, {YS[-1]:.1f}] m, step {STEP} m, "
      f"{len(XS)} x {len(YS)} = {XS.size * YS.size} IK calls per map, {t_ik:.0f} s for {len(X_BASES)} maps")
for x, C in maps.items():
    vals, cnt = np.unique(C, return_counts=True)
    reach = C > 0
    r = np.hypot(*np.meshgrid(XS - x, YS))
    print(f"\nbase at x = {x:+.1f} m: " + ", ".join(f"{v} IK: {c} pts" for v, c in zip(vals, cnt))
          + f";  reachable radius from axis 1: {r[reach].min():.3f} .. {r[reach].max():.3f} m")
C0 = maps[0.0]
shift = int(round(1.0 / STEP))
same = np.array_equal(maps[1.0][:, shift:], C0[:, :-shift]) and np.array_equal(maps[-1.0][:, :-shift], C0[:, shift:])
print(f"\nmap at x = +1 / -1 equals the map at x = 0 shifted by +1 / -1 m: {same}")
print("figures: results/figures/reach_map_x0.png, results/figures/reach_maps_x.png")
print("\nreading: the count is constant on regions bounded by the images of the three singular families.")
print("         Outer boundary: elbow family (arm stretched).  Inner hole of radius d4 = 0.174 m: shoulder family")
print("         (O5 cannot come closer to axis 1 than the arm-plane offset).  Inside r = 1.0 m all 8 branches")
print("         exist; in the ring 1.0 .. 1.24 m only one wrist choice keeps O4 within the planar 2R reach (4).")
print("         Moving the base translates the whole map: the slide adds reach only along x.")

sys.stdout = tee.out
out = HERE.parent / "results" / "step06_reach_map.md"
header = "# Step 6: reachable-position maps at discrete base positions" + chr(10) * 2 + "Script: `scripts/step06_reach_map.py`" + chr(10) * 2
img = "![reach map x = 0](figures/reach_map_x0.png)" + chr(10) * 2 + "![reach maps](figures/reach_maps_x.png)" + chr(10) * 2
fence = "```"
out.write_text(header + img + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
