"""Step 7B: one (s, x) map on which the base does not have to move.

Same path, grid and dot rule as step07_sx_map.py, one branch only.  A parked base is a horizontal line in
the map; it is a valid plan when every dot on that row is green and the joints are continuous along it.

Run from the repo root:   python scripts/step07b_sx_parked.py
Output: results/figures/sx_map_parked.png, results/step07b_sx_parked.md
"""
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e import ik_world
from jacobian_p6r import jacobian_reduced


class Tee:
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

# ------------------------------------------------ same definitions as step07_sx_map.py
Z = 0.9
CORNERS = np.array([[-0.8, 0.25], [0.8, 0.25], [0.8, 0.85], [-0.8, 0.85]])
R_FLANGE = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
S = np.linspace(0, 1, 111)
X = np.arange(-1.0, 1.0 + 1e-9, 0.05)
SIG_MIN = 0.03
DQ_MAX = 0.5
BRANCH = "S+ W- E+"
X_PARK = 0.0                                             # the parked base position drawn in red
# -----------------------------------------------------------------------------


def pose(s):
    seg = np.roll(CORNERS, -1, axis=0) - CORNERS
    L = np.linalg.norm(seg, axis=1)
    cum = np.r_[0, np.cumsum(L)] / L.sum()
    k = min(np.searchsorted(cum, s, side="right") - 1, 3)
    p = CORNERS[k] + (s - cum[k]) / (cum[k + 1] - cum[k]) * seg[k]
    T = np.eye(4)
    T[:3, :3] = R_FLANGE
    T[:3, 3] = [p[0], p[1], Z]
    return T


NS, NX = len(S), len(X)
ok = np.zeros((NS, NX), bool)
Q = np.full((NS, NX, 6), np.nan)
SIG = np.full((NS, NX), np.nan)
for i, s in enumerate(S):
    T = pose(s)
    for j, x in enumerate(X):
        for sol in ik_world(T, x):
            if sol["label"] == BRANCH:
                Q[i, j] = sol["q"]
                SIG[i, j] = smin(jacobian_reduced(np.r_[x, sol["q"]]))
                ok[i, j] = SIG[i, j] >= SIG_MIN

# rows on which a parked base completes the path: all green and continuous
rows = [j for j in range(NX) if ok[:, j].all() and np.abs(np.diff(Q[:, j], axis=0)).max() <= DQ_MAX]
jp = int(np.argmin(np.abs(X - X_PARK)))
assert jp in rows, f"x = {X_PARK} m is not a valid parked position on branch {BRANCH}"

# ------------------------------------------------ figure
GREEN, GREY, RED = "#2ca25f", "#d9d9d6", "#d62728"
SS, XX = np.meshgrid(S, X, indexing="ij")
fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
ax.axhspan(X[rows[0]] - 0.025, X[rows[-1]] + 0.025, color=RED, alpha=0.12, lw=0)
ax.scatter(SS[~ok], XX[~ok], s=9, c=GREY, linewidths=0)
ax.scatter(SS[ok], XX[ok], s=9, c=GREEN, linewidths=0)
ax.plot(S, np.full(NS, X[jp]), color=RED, lw=3.2)
for sc in np.cumsum(np.linalg.norm(np.roll(CORNERS, -1, axis=0) - CORNERS, axis=1))[:-1] / 4.4:
    ax.axvline(sc, color="#888", lw=0.6, ls=":")
ax.set_xlim(-0.01, 1.01)
ax.set_ylim(X[0] - 0.03, X[-1] + 0.03)
ax.set_xlabel("path progress s")
ax.set_ylabel("base position x [m]")
ax.set_title(f"branch {BRANCH}: the base parked at x = {X[jp]:+.2f} m completes the path", fontsize=10)
for sp in ax.spines.values():
    sp.set_color("#999")
handles = [Line2D([], [], ls="", marker="o", color=GREEN, ms=7, label=f"admissible root (sigma_min(J_red) >= {SIG_MIN})"),
           Line2D([], [], ls="", marker="o", color=GREY, ms=7, label="no admissible root"),
           Line2D([], [], color=RED, lw=3.2, label=f"parked base, x(s) = {X[jp]:+.2f} m"),
           Patch(color=RED, alpha=0.12, label=f"all parked x that work: {X[rows[0]]:+.2f} .. {X[rows[-1]]:+.2f} m")]
fig.legend(handles=handles, loc="lower center", ncol=2, fontsize=9, bbox_to_anchor=(0.5, -0.13))
fig_path = HERE.parent / "results" / "figures" / "sx_map_parked.png"
for attempt in range(5):
    try:
        fig.savefig(fig_path, dpi=170, bbox_inches="tight")
        break
    except OSError:
        time.sleep(2)

# ------------------------------------------------ report
travel = np.abs(np.diff(Q[:, jp], axis=0)).sum(axis=0)
print(f"=== (s, x) map of branch {BRANCH} with a parked base ===")
print(f"path: rectangle {CORNERS.tolist()} at z = {Z} m, tool down; {NS} path samples, x from {X[0]} to {X[-1]:.1f} m in {X[1] - X[0]:.2f} m steps")
print(f"parked base positions that complete the path (every dot green, joints continuous): x = {X[rows[0]]:+.2f} .. {X[rows[-1]]:+.2f} m ({len(rows)} rows)")
print(f"drawn: x = {X[jp]:+.2f} m")
print(f"  base travel 0 m, joint travel |dq|_2 summed = {np.linalg.norm(np.diff(Q[:, jp], axis=0), axis=1).sum():.2f} rad")
print(f"  joint travel per joint [deg] = {np.rad2deg(travel).round(0)}")
print(f"  joint ranges [deg]: " + ", ".join(f"q{k + 1} {np.rad2deg(Q[:, jp, k].min()):.0f}..{np.rad2deg(Q[:, jp, k].max()):.0f}" for k in range(6)))
print(f"  min sigma_min(J_red) along the path = {SIG[:, jp].min():.3f} at s = {S[int(np.argmin(SIG[:, jp]))]:.2f}")
far = np.hypot(CORNERS[:, 0] - X[jp], CORNERS[:, 1]).max()
print(f"why it works: from x = {X[jp]:+.2f} m the farthest corner is {far:.3f} m away, inside the 1.241 m reach of this wrist choice,")
print("              and the nearest point of the path is 0.25 m away, outside the shoulder hole of radius 0.174 m.")
print(f"figure: {fig_path.relative_to(HERE.parent)}")

sys.stdout = tee.out
out = HERE.parent / "results" / "step07b_sx_parked.md"
header = "# Step 7B: (s, x) map with a parked base" + chr(10) * 2 + "Script: `scripts/step07b_sx_parked.py`" + chr(10) * 2
img = "![sx map parked](figures/sx_map_parked.png)" + chr(10) * 2
fence = "```"
out.write_text(header + img + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
