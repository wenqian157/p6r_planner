"""Step 7: the path-redundancy map (s, x) of the P6R and a planned base schedule (lecture 8, slides 14-17).

Path:   a closed rectangle in the horizontal plane at flange height Z, tool axis pointing down, longer than
        the arm can reach from one base position, so the base has to move.  s in [0, 1] is arc length.
Map:    for every (s, x) on a grid, x in [-1, 1] m, run the analytical IK of the arm (x fixed) and classify
        each of the 8 branches:
            admissible         root exists, sigma_min(J_red) >= SIG_MIN
            chart-singular     root exists, sigma_min(J_red) <  SIG_MIN but sigma_min(J) >= SIG_MIN:
                               the fixed-x arm is singular, the whole body is not (slide 18)
            body-singular      root exists, both below SIG_MIN
            no root            that branch cannot reach the pose from this base position
Plan:   branch-aware: layered graph over s, nodes = admissible (branch, x), edges between neighbouring s
        for the same branch with |dx| <= DX_MAX and joint continuity (no +/- pi seam crossing),
        cost = joint travel + W_X * base travel; dynamic programming gives the cheapest complete schedule.
        Greedy baseline: base parked at the start x of that schedule, arm alone; stops at the first non-admissible s.

Run from the repo root:   python scripts/step07_sx_map.py     (about 1 min)
Output: results/figures/sx_map.png, results/step07_sx_map.md
"""
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e import ik_world
from jacobian_p6r import jacobian


class Tee:
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
pi = np.pi
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

# ------------------------------------------------ path and map definition
Z = 0.9                                                  # flange height [m]
CORNERS = np.array([[-0.8, 0.25], [0.8, 0.25], [0.8, 0.85], [-0.8, 0.85]])   # rectangle in (x_w, y_w) [m]
R_FLANGE = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)                 # tool axis pointing down
S = np.linspace(0, 1, 111)                               # path samples (dots, lecture-8 style)
X = np.arange(-1.0, 1.0 + 1e-9, 0.05)                    # base positions [m]
SIG_MIN = 0.03                                           # singularity threshold on sigma_min [m-ish units]
DX_MAX = 0.05                                            # max base travel per path step [m]
W_X = 2.0                                                # weight of base travel in the cost [rad/m]
X_START = -1.0                                           # the base starts here at s = 0 (None: free start)
DQ_MAX = 0.5                                             # max joint step for continuity [rad], unwrapped
LABELS = ["S+ W+ E+", "S+ W+ E-", "S+ W- E+", "S+ W- E-", "S- W+ E+", "S- W+ E-", "S- W- E+", "S- W- E-"]
# -----------------------------------------------------------------------------


def pose(s):
    """Flange pose on the closed rectangle at arc-length fraction s."""
    seg = np.roll(CORNERS, -1, axis=0) - CORNERS
    L = np.linalg.norm(seg, axis=1)
    cum = np.r_[0, np.cumsum(L)] / L.sum()
    k = min(np.searchsorted(cum, s, side="right") - 1, 3)
    p = CORNERS[k] + (s - cum[k]) / (cum[k + 1] - cum[k]) * seg[k]
    T = np.eye(4)
    T[:3, :3] = R_FLANGE
    T[:3, 3] = [p[0], p[1], Z]
    return T


# ------------------------------------------------ 1. the map
t0 = time.time()
NB, NS, NX = len(LABELS), len(S), len(X)
status = np.zeros((NB, NS, NX), int)            # 0 no root, 1 body-singular, 2 chart-singular, 3 admissible
Q = np.full((NB, NS, NX, 6), np.nan)
SIG = np.full((2, NB, NS, NX), np.nan)          # sigma_min of J_red and of J
for i, s in enumerate(S):
    T = pose(s)
    for j, x in enumerate(X):
        for sol in ik_world(T, x):
            b = LABELS.index(sol["label"])
            q7 = np.r_[x, sol["q"]]
            J = jacobian(q7)
            sr, sf = smin(J[:, 1:]), smin(J)
            Q[b, i, j], SIG[:, b, i, j] = sol["q"], (sr, sf)
            status[b, i, j] = 3 if sr >= SIG_MIN else (2 if sf >= SIG_MIN else 1)
t_map = time.time() - t0


# ------------------------------------------------ 2. branch-aware plan (dynamic programming per branch)
def plan(b):
    """Cheapest admissible schedule x(s) on branch b.  Returns (cost, [j_0..j_NS-1]) or None."""
    INF = np.inf
    adm = status[b] == 3
    start_ok = adm[0] if X_START is None else adm[0] & np.isclose(X, X_START)
    cost = np.where(start_ok, 0.0, INF)
    back = np.zeros((NS, NX), int)
    jmax = int(round(DX_MAX / (X[1] - X[0])))
    for i in range(1, NS):
        new = np.full(NX, INF)
        for j in np.flatnonzero(adm[i]):
            lo, hi = max(0, j - jmax), min(NX, j + jmax + 1)
            best, arg = INF, -1
            for jp in range(lo, hi):
                if cost[jp] == INF:
                    continue
                dq = Q[b, i, j] - Q[b, i - 1, jp]
                if np.abs(dq).max() > DQ_MAX:              # unwrapped: a +/- pi seam crossing is forbidden
                    continue
                c = cost[jp] + np.linalg.norm(dq) + W_X * abs(X[j] - X[jp])
                if c < best:
                    best, arg = c, jp
            new[j], back[i, j] = best, arg
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


def greedy(b, j0):
    """Base parked at X[j0], arm follows the path on branch b until the first non-admissible sample."""
    for i in range(NS):
        if status[b, i, j0] != 3:
            return i
        if i > 0 and np.abs(Q[b, i, j0] - Q[b, i - 1, j0]).max() > DQ_MAX:
            return i
    return NS


# ------------------------------------------------ 3. figure (lecture-8 style: one dot per IK evaluation)
GREEN, GREY, RED = "#2ca25f", "#d9d9d6", "#d62728"
SS, XX = np.meshgrid(S, X, indexing="ij")
fig, axes = plt.subplots(2, 4, figsize=(16, 7), constrained_layout=True, sharex=True, sharey=True)
for b, ax in enumerate(axes.ravel()):
    ok = status[b] == 3
    ax.scatter(SS[~ok], XX[~ok], s=6, c=GREY, marker="o", linewidths=0)
    ax.scatter(SS[ok], XX[ok], s=6, c=GREEN, marker="o", linewidths=0)
    ax.set_title(f"branch {LABELS[b]}", fontsize=10)
    for sc in np.r_[0, np.cumsum(np.linalg.norm(np.roll(CORNERS, -1, axis=0) - CORNERS, axis=1))][1:-1] / 4.4:
        ax.axvline(sc, color="#888", lw=0.6, ls=":")
    if b in feasible:
        ax.plot(S, X[feasible[b][1]], color=RED, lw=3.2 if b == b_best else 2.0, alpha=1.0 if b == b_best else 0.75)
    ax.set_xlim(-0.01, 1.01)
    ax.set_ylim(X[0] - 0.03, X[-1] + 0.03)
    ax.set_facecolor("white")
    for sp in ax.spines.values():
        sp.set_color("#999")
for ax in axes[1]:
    ax.set_xlabel("path progress s")
for ax in axes[:, 0]:
    ax.set_ylabel("base position x [m]")
handles = [Line2D([], [], ls="", marker="o", color=GREEN, ms=7, label=f"admissible root (sigma_min(J_red) >= {SIG_MIN})"),
           Line2D([], [], ls="", marker="o", color=GREY, ms=7, label="no admissible root"),
           Line2D([], [], color=RED, lw=3.2, label=f"planned base schedule x(s) from x = {X_START} m, best branch (thin: other branches)")]
fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=9, bbox_to_anchor=(0.5, -0.05))
fig.suptitle(f"Path-redundancy map (s, x): every dot is one IK evaluation; rectangle {CORNERS.tolist()} at z = {Z} m, "
             f"tool down; dotted = corners", fontsize=10)
fig_path = HERE.parent / "results" / "figures" / "sx_map.png"
fig_path.parent.mkdir(parents=True, exist_ok=True)
for attempt in range(5):                                  # a viewer holding the PNG open blocks the write briefly
    try:
        fig.savefig(fig_path, dpi=170, bbox_inches="tight")
        break
    except OSError:
        time.sleep(2)
else:
    fig_path = fig_path.with_name("sx_map_new.png")
    fig.savefig(fig_path, dpi=170, bbox_inches="tight")

# ------------------------------------------------ 4. report
print("=== path-redundancy map (s, x) ===")
print(f"path: closed rectangle through {CORNERS.tolist()} (x_w, y_w) at z = {Z} m, tool axis down, perimeter 4.4 m, {NS} samples")
print(f"base: x in [{X[0]}, {X[-1]}] m, step {X[1] - X[0]:.3f} m, {NX} positions;  IK + Jacobians at {NS * NX} (s, x) points in {t_map:.0f} s")
print(f"thresholds: sigma_min >= {SIG_MIN} admissible;  plan: start x = {X_START} m, |dx| <= {DX_MAX} m/step, |dq| <= {DQ_MAX} rad/step (unwrapped), cost = |dq| + {W_X}|dx|")
print(f"\n{'branch':10s} {'admissible':>10s} {'chart-sing':>10s} {'body-sing':>10s} {'no root':>8s}   {'legal at s=0':>12s} {'plan cost':>9s}  x range of plan")
for b in range(NB):
    cnt = [(status[b] == k).mean() * 100 for k in (3, 2, 1, 0)]
    n0 = int((status[b, 0] == 3).sum())
    pl = plans[b]
    tail = f"{pl[0]:9.2f}  [{X[pl[1]].min():+.2f}, {X[pl[1]].max():+.2f}] m" if pl else "     none  -"
    print(f"{LABELS[b]:10s} " + " ".join(f"{c:9.1f}%" for c in cnt) + f"   {n0:12d} {tail}")
for b in range(NB):
    if plans[b] is None:
        dq = np.abs(np.diff(Q[b], axis=0))                       # (NS-1, NX, 6), nan where no root
        both = (status[b, 1:] == 3) & (status[b, :-1] == 3)       # consecutive admissible samples at the same x
        hit = {f"q{k + 1}" for k in range(6) if np.any(both & (dq[..., k] > DQ_MAX))}
        print(f"\nbranch {LABELS[b]} has no complete schedule: joints {sorted(hit)} cross the +/- pi seam along the path,")
        print("  which the +/- pi joint limits forbid.  The joint-limit effect appears as a path constraint, not in the map colours.")
if b_best is not None:
    c, route = feasible[b_best]
    qs = Q[b_best, np.arange(NS), route]
    travel = np.abs(np.diff(qs, axis=0)).sum(axis=0)
    print(f"\nbest branch {LABELS[b_best]}: cost {c:.2f}, base from x = {X[route[0]]:+.3f} to {X[route[-1]]:+.3f} m, "
          f"base travel {np.abs(np.diff(X[route])).sum():.2f} m, joint travel per joint [deg] = {np.rad2deg(travel).round(0)}")
    print(f"  min sigma_min(J_red) along the plan = {SIG[0, b_best, np.arange(NS), route].min():.3f}")
print(f"\nfigure: {fig_path.relative_to(HERE.parent)}")
print()
print("reading: each panel is one branch; a green dot means that branch holds the pose with the base at x and the")
print("         fixed-x arm is regular.  Grey: no root, or root at a singular configuration.  A horizontal line is a")
print("         parked base; the rectangle is longer than the arm reach, so no horizontal line stays green: the base must move.")

sys.stdout = tee.out
out = HERE.parent / "results" / "step07_sx_map.md"
header = "# Step 7: path-redundancy map (s, x) and base schedule" + chr(10) * 2 + "Script: `scripts/step07_sx_map.py`" + chr(10) * 2
img = "![sx map](figures/sx_map.png)" + chr(10) * 2
fence = "```"
out.write_text(header + img + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
