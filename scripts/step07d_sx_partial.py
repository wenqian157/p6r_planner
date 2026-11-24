"""Step 7D: the (s, x) map with the partial schedules of the branches that do not finish.

Same path, grid, thresholds and planner as step07_sx_map.py (horizontal) and step07c_sx_tilted.py (tilted).
A branch with a complete schedule is drawn as in those steps.  A branch without one is drawn as far as it
gets: the cheapest schedule that reaches the last sample the planner can reach on that branch, ending in a
cross where it stops, with the reason.  Dots are green (admissible root) or grey (everything else).

Run from the repo root:   python scripts/step07d_sx_partial.py            horizontal rectangle
                          python scripts/step07d_sx_partial.py 30 20      tilted 30 deg about x, 20 deg about y
Output: results/figures/sx_map_partial.png  or  sx_map_partial_x30y20.png      (about 1 min)
"""
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e import ik_world
from jacobian_p6r import jacobian_reduced

sys.stdout.reconfigure(encoding="utf-8")
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

# ------------------------------------------------ same definitions as steps 7 and 7C
TILT_X, TILT_Y = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) == 3 else (0.0, 0.0)
PIVOT = np.array([0.0, 0.25, 0.9])                       # world position of rectangle point (u, v) = (0, 0)
UV = np.array([[-0.8, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6]])
R_FLAT = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
S = np.linspace(0, 1, 111)
X = np.arange(-1.0, 1.0 + 1e-9, 0.05)
SIG_MIN, DX_MAX, W_X, X_START, DQ_MAX = 0.03, 0.05, 2.0, -1.0, 0.5
LABELS = ["S+ W+ E+", "S+ W+ E-", "S+ W- E+", "S+ W- E-", "S- W+ E+", "S- W+ E-", "S- W- E+", "S- W- E-"]
cx, sx_, cy, sy_ = (f(np.deg2rad(a)) for a in (TILT_X, TILT_Y) for f in (np.cos, np.sin))
R_TILT = np.array([[1, 0, 0], [0, cx, -sx_], [0, sx_, cx]]) @ np.array([[cy, 0, -sy_], [0, 1, 0], [sy_, 0, cy]])
R_FLANGE = R_TILT @ R_FLAT
SEG = np.roll(UV, -1, axis=0) - UV
CUM = np.r_[0, np.cumsum(np.linalg.norm(SEG, axis=1))] / np.linalg.norm(SEG, axis=1).sum()
TILTED = bool(TILT_X or TILT_Y)


def pose(s):
    k = min(np.searchsorted(CUM, s, side="right") - 1, 3)
    uv = UV[k] + (s - CUM[k]) / (CUM[k + 1] - CUM[k]) * SEG[k]
    T = np.eye(4)
    T[:3, :3] = R_FLANGE
    T[:3, 3] = PIVOT + R_TILT @ np.array([uv[0], uv[1], 0.0])
    return T


# ------------------------------------------------ 1. the map
NB, NS, NX = len(LABELS), len(S), len(X)
adm = np.zeros((NB, NS, NX), bool)
Q = np.full((NB, NS, NX, 6), np.nan)
for i, s in enumerate(S):
    T = pose(s)
    for j, x in enumerate(X):
        for sol in ik_world(T, x):
            b = LABELS.index(sol["label"])
            Q[b, i, j] = sol["q"]
            adm[b, i, j] = smin(jacobian_reduced(np.r_[x, sol["q"]])) >= SIG_MIN


# ------------------------------------------------ 2. plan; a branch that cannot finish returns its partial schedule
def plan(b):
    """Returns (complete, cost, route, reason).  route has one x index per sample it reaches (empty: no start)."""
    INF = np.inf
    cost = np.where(adm[b, 0] & np.isclose(X, X_START), 0.0, INF)
    if np.all(cost == INF):
        return False, INF, [], f"no admissible root at the start x = {X_START:g} m"
    back = np.zeros((NS, NX), int)
    jmax = int(round(DX_MAX / (X[1] - X[0])))
    last, reason = 0, ""
    for i in range(1, NS):
        new = np.full(NX, INF)
        seam, jump = np.zeros(6, int), np.zeros(6, int)   # rejected steps: +/- pi crossing, or a large joint step
        for j in np.flatnonzero(adm[b, i]):
            for jp in range(max(0, j - jmax), min(NX, j + jmax + 1)):
                if cost[jp] == INF:
                    continue
                q0, q1 = Q[b, i - 1, jp], Q[b, i, j]
                dq = q1 - q0
                if np.abs(dq).max() > DQ_MAX:
                    k = int(np.argmax(np.abs(dq)))
                    (seam if (q0[k] * q1[k] < 0 and min(abs(q0[k]), abs(q1[k])) > 2.5) else jump)[k] += 1
                    continue
                c = cost[jp] + np.linalg.norm(dq) + W_X * abs(X[j] - X[jp])
                if c < new[j]:
                    new[j], back[i, j] = c, jp
        if np.all(new == INF):
            if seam.sum() >= jump.sum() and seam.any():
                reason = f"q{int(np.argmax(seam)) + 1} reaches ±180°"
            elif jump.any():
                reason = f"q{int(np.argmax(jump)) + 1} would jump"
            else:
                reason = "no admissible root ahead"
            break
        cost, last = new, i
    j = int(np.argmin(cost))
    total = cost[j]
    route = [j]
    for i in range(last, 0, -1):
        j = back[i, j]
        route.append(j)
    return last == NS - 1, total, route[::-1], reason


plans = [plan(b) for b in range(NB)]
done = [b for b in range(NB) if plans[b][0]]
b_best = min(done, key=lambda b: plans[b][1]) if done else None

# ------------------------------------------------ 3. figure (same style as step 7)
GREEN, GREY, RED, DARK = "#2ca25f", "#d9d9d6", "#d62728", "#222222"
SS, XX = np.meshgrid(S, X, indexing="ij")
BOX = dict(boxstyle="round,pad=0.25", fc="white", ec="#999", lw=0.6)
fig, axes = plt.subplots(2, 4, figsize=(16, 7), constrained_layout=True, sharex=True, sharey=True)
for b, ax in enumerate(axes.ravel()):
    ok = adm[b]
    ax.scatter(SS[~ok], XX[~ok], s=6, c=GREY, marker="o", linewidths=0)
    ax.scatter(SS[ok], XX[ok], s=6, c=GREEN, marker="o", linewidths=0)
    for sc in CUM[1:-1]:
        ax.axvline(sc, color="#888", lw=0.6, ls=":")
    complete, cost, route, reason = plans[b]
    n = len(route)
    if n:
        ax.plot(S[:n], X[route], color=RED, lw=3.2 if b == b_best else 2.0, alpha=1.0 if b == b_best else 0.75)
    if complete:
        ax.set_title(f"branch {LABELS[b]}", fontsize=10)
    else:
        s_stop, x_stop = (S[n - 1], X[route[-1]]) if n else (0.0, X_START)
        ax.plot([s_stop], [x_stop], marker="X", ms=11, color=DARK, mec="white", mew=0.8, zorder=5, clip_on=False)
        text = (f"stops at s = {s_stop:.2f}" + chr(10) + reason) if n else ("cannot start:" + chr(10) + reason)
        tx = s_stop + 0.06 if s_stop < 0.55 else s_stop - 0.45
        ty = x_stop - 0.42 if x_stop > -0.3 else x_stop + 0.32
        ax.annotate(text, (s_stop, x_stop), xytext=(tx, ty), fontsize=9, color=DARK,
                    arrowprops=dict(arrowstyle="-", color=DARK, lw=0.8), bbox=BOX, zorder=6)
        ax.set_title(f"branch {LABELS[b]}  (does not finish)", fontsize=10)
    ax.set_xlim(-0.01, 1.01)
    ax.set_ylim(X[0] - 0.03, X[-1] + 0.03)
    for sp in ax.spines.values():
        sp.set_color("#999")
for ax in axes[1]:
    ax.set_xlabel("path progress s")
for ax in axes[:, 0]:
    ax.set_ylabel("base position x [m]")
handles = [Line2D([], [], ls="", marker="o", color=GREEN, ms=7, label=f"admissible root (sigma_min(J_red) >= {SIG_MIN})"),
           Line2D([], [], ls="", marker="o", color=GREY, ms=7, label="no admissible root"),
           Line2D([], [], color=RED, lw=3.2, label=f"planned base schedule x(s) from x = {X_START} m, best branch (thin: other branches)"),
           Line2D([], [], ls="", marker="X", color=DARK, ms=10, mec="white", label="the schedule stops here")]
fig.legend(handles=handles, loc="lower center", ncol=2, fontsize=9, bbox_to_anchor=(0.5, -0.09))
if TILTED:
    tilt_txt = " and ".join(f"{a:g} deg about world {n}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a)
    title = f"Path-redundancy map (s, x), rectangle on a plane tilted {tilt_txt}, tool normal to the plane; dotted = corners"
else:
    corners = (PIVOT[:2] + UV).tolist()
    title = (f"Path-redundancy map (s, x): every dot is one IK evaluation; rectangle {corners} at z = {PIVOT[2]} m, "
             f"tool down; dotted = corners")
fig.suptitle(title, fontsize=10)
tag = "".join(f"{n}{a:g}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a)
fig_path = HERE.parent / "results" / "figures" / ("sx_map_partial" + ("_" + tag if tag else "") + ".png")
for attempt in range(5):                                  # a viewer holding the PNG open blocks the write briefly
    try:
        fig.savefig(fig_path, dpi=170, bbox_inches="tight")
        break
    except OSError:
        time.sleep(2)

# ------------------------------------------------ 4. summary
print(f"{'branch':10s} {'result':>10s} {'reaches s':>10s} {'x there':>8s} {'cost':>7s}  stops because")
for b in range(NB):
    complete, cost, route, reason = plans[b]
    n = len(route)
    if n == 0:
        print(f"{LABELS[b]:10s} {'no start':>10s} {'-':>10s} {'-':>8s} {'-':>7s}  {reason}")
        continue
    print(f"{LABELS[b]:10s} {'complete' if complete else 'stops':>10s} {S[n - 1]:10.3f} {X[route[-1]]:+8.2f} {cost:7.2f}  {reason}")
print(f"best branch: {LABELS[b_best] if b_best is not None else 'none'}")
print(f"figure: {fig_path.relative_to(HERE.parent)}")
