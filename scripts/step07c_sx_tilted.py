"""Step 7C: the (s, x) map for a path on a tilted plane.

Same rectangle, grid, dot rule and planner as step07_sx_map.py, but the plane of the rectangle is rotated
by TILT_X deg about world x and TILT_Y deg about world y, and the tool axis stays normal to that plane:
    axis 'x':  the plane rises away from the rail (far long side higher), tool leans toward +y
    axis 'y':  the plane rises along the rail (the +x end higher),        tool leans toward +x
    0 deg reproduces the horizontal path of step 7.
Rectangle coordinates (u, v): u in [-0.8, 0.8] m along the long side, v in [0, 0.6] m; world point
p = PIVOT + R_tilt @ (u, v, 0).

Run from the repo root:   python scripts/step07c_sx_tilted.py            (axis x, 30 deg)
                          python scripts/step07c_sx_tilted.py y 20       (axis, angle [deg])
                          python scripts/step07c_sx_tilted.py xy 30 20   (both: deg about x, deg about y)
Output: results/figures/sx_map_tilt_<axis><deg>.png, results/step07c_sx_tilted_<axis><deg>.md
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
from jacobian_p6r import jacobian


class Tee:
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

# ------------------------------------------------ path and map definition
TILT_X, TILT_Y = 30.0, 0.0                               # deg about world x (rise away from the rail), about world y (rise along it)
if len(sys.argv) == 3:
    TILT_X, TILT_Y = (float(sys.argv[2]), 0.0) if sys.argv[1] == "x" else (0.0, float(sys.argv[2]))
elif len(sys.argv) == 4:                                 # xy <deg about x> <deg about y>
    TILT_X, TILT_Y = float(sys.argv[2]), float(sys.argv[3])
PIVOT = np.array([0.0, 0.25, 0.9])                       # world position of rectangle point (u, v) = (0, 0)
UV = np.array([[-0.8, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6]])
R_FLAT = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)   # tool axis down for the horizontal plane
S = np.linspace(0, 1, 111)
X = np.arange(-1.0, 1.0 + 1e-9, 0.05)
SIG_MIN = 0.03
DX_MAX, DQ_MAX, W_X = 0.05, 0.5, 2.0
X_START = -1.0                                           # base position at s = 0 (None: free start)
LABELS = ["S+ W+ E+", "S+ W+ E-", "S+ W- E+", "S+ W- E-", "S- W+ E+", "S- W+ E-", "S- W- E+", "S- W- E-"]
# -----------------------------------------------------------------------------
cx, sx_, cy, sy_ = (f(np.deg2rad(a)) for a in (TILT_X, TILT_Y) for f in (np.cos, np.sin))
R_ABOUT_X = np.array([[1, 0, 0], [0, cx, -sx_], [0, sx_, cx]])          # far long side rises
R_ABOUT_Y = np.array([[cy, 0, -sy_], [0, 1, 0], [sy_, 0, cy]])          # +u end rises
R_TILT = R_ABOUT_X @ R_ABOUT_Y
TILT_TXT = " and ".join(f"{a:g} deg about world {n}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a) or "0 deg"
R_FLANGE = R_TILT @ R_FLAT
SEG = np.roll(UV, -1, axis=0) - UV
CUM = np.r_[0, np.cumsum(np.linalg.norm(SEG, axis=1))] / np.linalg.norm(SEG, axis=1).sum()


def pose(s):
    k = min(np.searchsorted(CUM, s, side="right") - 1, 3)
    uv = UV[k] + (s - CUM[k]) / (CUM[k + 1] - CUM[k]) * SEG[k]
    T = np.eye(4)
    T[:3, :3] = R_FLANGE
    T[:3, 3] = PIVOT + R_TILT @ np.array([uv[0], uv[1], 0.0])
    return T


# ------------------------------------------------ 1. the map
t0 = time.time()
NB, NS, NX = len(LABELS), len(S), len(X)
status = np.zeros((NB, NS, NX), int)            # 0 no root, 1 body-singular, 2 chart-singular, 3 admissible
Q = np.full((NB, NS, NX, 6), np.nan)
SIG = np.full((NB, NS, NX), np.nan)
for i, s in enumerate(S):
    T = pose(s)
    for j, x in enumerate(X):
        for sol in ik_world(T, x):
            b = LABELS.index(sol["label"])
            J = jacobian(np.r_[x, sol["q"]])
            sr, sf = smin(J[:, 1:]), smin(J)
            Q[b, i, j], SIG[b, i, j] = sol["q"], sr
            status[b, i, j] = 3 if sr >= SIG_MIN else (2 if sf >= SIG_MIN else 1)
t_map = time.time() - t0


# ------------------------------------------------ 2. plan per branch (same dynamic programming as step 7)
def plan(b):
    INF = np.inf
    adm = status[b] == 3
    start_ok = adm[0] if X_START is None else adm[0] & np.isclose(X, X_START)
    cost = np.where(start_ok, 0.0, INF)
    back = np.zeros((NS, NX), int)
    jmax = int(round(DX_MAX / (X[1] - X[0])))
    for i in range(1, NS):
        new = np.full(NX, INF)
        for j in np.flatnonzero(adm[i]):
            for jp in range(max(0, j - jmax), min(NX, j + jmax + 1)):
                if cost[jp] == INF:
                    continue
                dq = Q[b, i, j] - Q[b, i - 1, jp]
                if np.abs(dq).max() > DQ_MAX:
                    continue
                cc = cost[jp] + np.linalg.norm(dq) + W_X * abs(X[j] - X[jp])
                if cc < new[j]:
                    new[j], back[i, j] = cc, jp
        cost = new
    if np.all(cost == INF):
        return None
    j = int(np.argmin(cost))
    route = [j]
    for i in range(NS - 1, 0, -1):
        j = back[i, j]
        route.append(j)
    return cost.min(), route[::-1]


plans = {b: plan(b) for b in range(NB)}
feasible = {b: p for b, p in plans.items() if p is not None}
b_best = min(feasible, key=lambda b: feasible[b][0]) if feasible else None
parked = {b: [j for j in range(NX) if (status[b, :, j] == 3).all()
              and np.abs(np.diff(Q[b, :, j], axis=0)).max() <= DQ_MAX] for b in range(NB)}

# ------------------------------------------------ 3. figure
GREEN, GREY, ORANGE, RED = "#2ca25f", "#d9d9d6", "#f28e2b", "#d62728"
SS, XX = np.meshgrid(S, X, indexing="ij")
n_sing = int(((status == 1) | (status == 2)).sum())
fig, axes = plt.subplots(2, 4, figsize=(16, 7), constrained_layout=True, sharex=True, sharey=True)
for b, ax in enumerate(axes.ravel()):
    ok, sing = status[b] == 3, (status[b] == 1) | (status[b] == 2)
    ax.scatter(SS[status[b] == 0], XX[status[b] == 0], s=6, c=GREY, linewidths=0)
    ax.scatter(SS[ok], XX[ok], s=6, c=GREEN, linewidths=0)
    ax.scatter(SS[sing], XX[sing], s=9, c=ORANGE, linewidths=0)
    ax.set_title(f"branch {LABELS[b]}", fontsize=10)
    for sc in CUM[1:-1]:
        ax.axvline(sc, color="#888", lw=0.6, ls=":")
    if b in feasible:
        ax.plot(S, X[feasible[b][1]], color=RED, lw=3.2 if b == b_best else 2.0, alpha=1.0 if b == b_best else 0.75)
    ax.set_xlim(-0.01, 1.01)
    ax.set_ylim(X[0] - 0.03, X[-1] + 0.03)
    for sp in ax.spines.values():
        sp.set_color("#999")
for ax in axes[1]:
    ax.set_xlabel("path progress s")
for ax in axes[:, 0]:
    ax.set_ylabel("base position x [m]")
handles = [Line2D([], [], ls="", marker="o", color=GREEN, ms=7, label=f"admissible root (sigma_min(J_red) >= {SIG_MIN})"),
           Line2D([], [], ls="", marker="o", color=GREY, ms=7, label="no root")]
if n_sing:
    handles.append(Line2D([], [], ls="", marker="o", color=ORANGE, ms=7, label="root at a singular configuration"))
handles.append(Line2D([], [], color=RED, lw=3.2, label=f"planned base schedule x(s) from x = {X_START} m (thick: best branch)"))
fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.05))
fig.suptitle(f"Path-redundancy map (s, x), rectangle on a plane tilted {TILT_TXT}, "
             f"tool normal to the plane; dotted = corners", fontsize=10)
tag = "".join(f"{n}{a:g}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a) or "0"
fig_path = HERE.parent / "results" / "figures" / f"sx_map_tilt_{tag}.png"
for attempt in range(5):
    try:
        fig.savefig(fig_path, dpi=170, bbox_inches="tight")
        break
    except OSError:
        time.sleep(2)

# ------------------------------------------------ 4. report
corners_w = np.array([PIVOT + R_TILT @ np.array([u, v, 0.0]) for u, v in UV])
print(f"=== (s, x) map, rectangle on a plane tilted {TILT_TXT} ===")
print("corners in the world [m]:")
for cw in corners_w:
    print(f"   ({cw[0]:+.3f}, {cw[1]:+.3f}, {cw[2]:+.3f})")
print(f"tool axis (flange outward normal) in the world = {np.round(R_FLANGE[:, 0], 3)}   (normal to the plane, pointing into it)")
print(f"grid: {NS} path samples x {NX} base positions, {t_map:.0f} s;  start x = {X_START} m, cost = |dq| + {W_X}|dx|")
q5 = np.abs(np.rad2deg(Q[..., 4]))
print(f"|q5| over all roots: {np.nanmin(q5):.0f} .. {np.nanmax(q5):.0f} deg   (wrist family at 0 and 180)")
print(f"\n{'branch':10s} {'admissible':>10s} {'singular':>9s} {'no root':>8s}   {'plan cost':>9s}  {'x range of plan':22s} parked x that work")
for b in range(NB):
    a, sg, nr = [(m.mean() * 100) for m in (status[b] == 3, (status[b] == 1) | (status[b] == 2), status[b] == 0)]
    pl = plans[b]
    ptxt = f"{pl[0]:9.2f}  [{X[pl[1]].min():+.2f}, {X[pl[1]].max():+.2f}] m      " if pl else "     none  -                     "
    pk = f"{X[parked[b][0]]:+.2f} .. {X[parked[b][-1]]:+.2f} m" if parked[b] else "none"
    print(f"{LABELS[b]:10s} {a:9.1f}% {sg:8.1f}% {nr:7.1f}%   {ptxt} {pk}")
for b in range(NB):
    if plans[b] is None:
        if X_START is not None and not (status[b, 0] == 3)[np.isclose(X, X_START)].any():
            print(f"\nbranch {LABELS[b]}: no schedule, the start x = {X_START} m is not admissible at s = 0")
            continue
        dq = np.abs(np.diff(Q[b], axis=0))
        both = (status[b, 1:] == 3) & (status[b, :-1] == 3)
        hit = sorted({f"q{k + 1}" for k in range(6) if np.any(both & (dq[..., k] > DQ_MAX))})
        print(f"\nbranch {LABELS[b]}: no schedule" + (f", joints {hit} cross the +/- pi seam along the path" if hit else ", the admissible region is not connected from the start"))
if b_best is not None:
    cst, route = feasible[b_best]
    qs = Q[b_best, np.arange(NS), route]
    print(f"\nbest branch {LABELS[b_best]}: cost {cst:.2f}, base from x = {X[route[0]]:+.2f} to {X[route[-1]]:+.2f} m, "
          f"base travel {np.abs(np.diff(X[route])).sum():.2f} m")
    print(f"  joint travel per joint [deg] = {np.rad2deg(np.abs(np.diff(qs, axis=0)).sum(axis=0)).round(0)}")
    print(f"  min sigma_min(J_red) along the plan = {SIG[b_best, np.arange(NS), route].min():.3f}")
print(f"\nsingular roots (orange): {n_sing} dots, of which whole-body singular: {int((status == 1).sum())}")
print(f"figure: {fig_path.relative_to(HERE.parent)}")

sys.stdout = tee.out
out = HERE.parent / "results" / f"step07c_sx_tilted_{tag}.md"
header = f"# Step 7C: (s, x) map, plane tilted {TILT_TXT}" + chr(10) * 2 + "Script: `scripts/step07c_sx_tilted.py`" + chr(10) * 2
img = f"![sx map tilted](figures/{fig_path.name})" + chr(10) * 2
fence = "```"
out.write_text(header + img + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
