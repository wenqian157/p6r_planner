"""DRAFT figures for Report section 3 (run from the repo root:  python drafts/ik_figures.py)
  drafts/figures/ik_all_solutions.png   the 8 IK branches of one tool pose (x = 0.5 m)
  drafts/figures/ik_branches_vs_x.png   same world pose, base slid along x: which branches exist
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e_full import *                         # solver + her modules
from ik_ur12e_full import _dh
from fk_dh import T_BASE, T_TOOL
from umr_params import MOUNT_XYZ, MOUNT_RPY, UMR_LENGTH, UMR_WIDTH, UMR_HEIGHT

OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)


def joint_points(x, q):
    """World positions of base, O1..O6 and flange for the arm at base position x."""
    T = origin([x, 0, 0], [0, 0, 0]) @ origin(MOUNT_XYZ, MOUNT_RPY) @ T_BASE
    P = [T[:3, 3]]
    for qi, (d, a, al) in zip(q, DH_ROWS):
        # split each DH row in two (after Tz(d), after Tx(a)) so the drawn links follow the real offsets
        T = T @ _dh(np.array(qi), d, 0.0, 0.0); P.append(T[:3, 3])
        T = T @ _dh(np.array(0.0), 0.0, a, al); P.append(T[:3, 3])
    return np.array(P)


def draw_base(ax, x):
    L, W, H = UMR_LENGTH / 2, UMR_WIDTH / 2, UMR_HEIGHT
    bx = np.array([[-L, -W], [L, -W], [L, W], [-L, W], [-L, -W]]) + [x, 0]
    for z in (0, H):
        ax.plot(bx[:, 0], bx[:, 1], z, color="0.6", lw=1)
    for cx, cy in bx[:4]:
        ax.plot([cx, cx], [cy, cy], [0, H], color="0.6", lw=1)


x = 0.5
T_star = fk_p6r(np.r_[x, np.deg2rad([10, -60, 90, -120, -90, 30])])
qW, sW = ik_world(T_star, x)

fig = plt.figure(figsize=(16, 8.5))
for b in range(8):
    ax = fig.add_subplot(2, 4, b + 1, projection="3d")
    draw_base(ax, x)
    if sW[0, b] == OK:
        P = joint_points(x, qW[0, b])
        ax.plot(*P.T, "-o", lw=3, ms=3, color="C0")
    p, z = T_star[:3, 3], T_star[:3, 2]
    ax.scatter(*p, color="C3", s=30)
    ax.quiver(*p, *(0.2 * z), color="C3")
    ax.set_xlim(-0.3, 1.7); ax.set_ylim(-1, 1); ax.set_zlim(0, 1.6)
    ax.set_box_aspect((1, 1, 0.8)); ax.view_init(22, -60)
    q = np.rad2deg(qW[0, b])
    ax.set_title(f"{LABELS[b]}\n" + (" ".join(f"{v:.0f}" for v in q) + " deg" if sW[0, b] == OK else STATUS[sW[0, b]]),
                 fontsize=9)
    ax.tick_params(labelsize=6)
fig.suptitle("All 8 analytical IK branches for one flange pose (red), base at x = 0.5 m", fontsize=12)
fig.tight_layout(); fig.savefig(OUT / "ik_all_solutions.png", dpi=130)

# ---- branches vs base position (same world target)
xs = np.linspace(-1.5, 1.5, 601)
S = ik_6r(np.stack([T06_from_world(T_star, xx) for xx in xs]))[1]
fig, ax = plt.subplots(figsize=(10, 3.4))
ax.imshow((S == OK).T, aspect="auto", cmap="Greens", vmin=-0.3, vmax=1.2, interpolation="nearest",
          extent=[xs[0], xs[-1], 7.5, -0.5])
ax.set_yticks(range(8)); ax.set_yticklabels(LABELS, fontsize=8)
ax.axvline(x, color="C3", ls="--", lw=1); ax.text(x + 0.02, -0.7, "x of section 1-2", color="C3", fontsize=8)
ax.set_xlabel("base position x [m]"); ax.set_title("Same world flange pose: which IK branches exist as the base slides (green = real IK)")
fig.tight_layout(); fig.savefig(OUT / "ik_branches_vs_x.png", dpi=130)
for b in range(8):
    xb = xs[S[:, b] == OK]
    print(f"{LABELS[b]}: real IK for x in [{xb.min():+.3f}, {xb.max():+.3f}] m" if len(xb) else f"{LABELS[b]}: never")
print("saved", OUT / "ik_all_solutions.png", "and", OUT / "ik_branches_vs_x.png")
