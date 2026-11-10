"""DRAFT (Report section 5, additions) - fixed-x baseline, greedy vs branch-aware planning, execution profiles.

Builds on scripts/step07_sx_map.py (same path, same grid, same thresholds, same DP planner) and adds what the
proposal asks for in section 5 and the step-07 script does not cover yet:

  5A  fixed-x baseline: start a continuation from every admissible root at s = 0 with the base parked at
      x = 0 m and at x = -1 m; record where each branch completes or fails, and why
      (no root / singular / +-pi seam = joint limit / continuity jump).
  5B  greedy continuation with x chosen locally (the proposal's "greedy"), two local rules:
        G-cost   : at each step the admissible neighbour with the smallest step cost |dq| + W_X |dx|
                   (the same cost the DP minimises, but one step at a time)
        G-margin : at each step the admissible neighbour with the largest sigma_min(J_red)
      both under the same constraints as the DP (|dx| <= DX_MAX, |dq| <= DQ_MAX unwrapped, same branch).
  5C  profiles along the executions: x(s), joints, distance to the +-pi seam, Cartesian residual (FK of the
      executed configuration vs the path pose), sigma_min(J_red) and sigma_min(J).

Run from the repo root:   python drafts/section5_planning.py            (about 20 s)
If the repo is not yet pulled, point it at a checkout of origin/main:   P6R_ROOT=/path/to/checkout python ...
Outputs: drafts/figures/s5_*.png, stdout (save as drafts/section5_output.txt)
"""
import os
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("P6R_ROOT", HERE.parent))
sys.path.insert(0, str(ROOT / "scripts"))
from ik_ur12e import ik_world                  # her analytical IK (step 5)
from jacobian_p6r import jacobian
from fk_urdf_p6r import fk as fk_p6r           # independent model for the Cartesian residual

OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
pi = np.pi
np.set_printoptions(precision=3, suppress=True, linewidth=140)
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

# ------------------------------------------------ settings: identical to scripts/step07_sx_map.py
Z = 0.9
CORNERS = np.array([[-0.8, 0.25], [0.8, 0.25], [0.8, 0.85], [-0.8, 0.85]])
R_FLANGE = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
S = np.linspace(0, 1, 111)
SIG_MIN, DX_MAX, W_X, X_START, DQ_MAX = 0.03, 0.05, 2.0, -1.0, 0.5
# optional overrides for the base-speed study (defaults = step 07):  S5_DX = max base step [m], S5_XSTEP = x grid step [m]
DX_MAX = float(os.environ.get("S5_DX", DX_MAX))
X = np.arange(-1.0, 1.0 + 1e-9, float(os.environ.get("S5_XSTEP", 0.05)))
TAG = os.environ.get("S5_TAG", "")
LABELS = ["S+ W+ E+", "S+ W+ E-", "S+ W- E+", "S+ W- E-", "S- W+ E+", "S- W+ E-", "S- W- E+", "S- W- E-"]
NB, NS, NX = len(LABELS), len(S), len(X)
JMAX = int(round(DX_MAX / (X[1] - X[0])))
PERIM = np.linalg.norm(np.roll(CORNERS, -1, 0) - CORNERS, axis=1).sum()
S_CORNERS = np.r_[0, np.cumsum(np.linalg.norm(np.roll(CORNERS, -1, 0) - CORNERS, axis=1))][1:-1] / PERIM


def pose(s):
    seg = np.roll(CORNERS, -1, axis=0) - CORNERS
    L = np.linalg.norm(seg, axis=1)
    cum = np.r_[0, np.cumsum(L)] / L.sum()
    k = min(np.searchsorted(cum, s, side="right") - 1, 3)
    p = CORNERS[k] + (s - cum[k]) / (cum[k + 1] - cum[k]) * seg[k]
    T = np.eye(4); T[:3, :3] = R_FLANGE; T[:3, 3] = [p[0], p[1], Z]
    return T


# ------------------------------------------------ the (s, x) map (same as step 07)
t0 = time.time()
status = np.zeros((NB, NS, NX), int)          # 0 no root, 1 body-singular, 2 chart-singular, 3 admissible
Q = np.full((NB, NS, NX, 6), np.nan)
SIG = np.full((2, NB, NS, NX), np.nan)
POSES = [pose(s) for s in S]
for i in range(NS):
    for j, x in enumerate(X):
        for sol in ik_world(POSES[i], x):
            b = LABELS.index(sol["label"])
            J = jacobian(np.r_[x, sol["q"]])
            sr, sf = smin(J[:, 1:]), smin(J)
            Q[b, i, j], SIG[:, b, i, j] = sol["q"], (sr, sf)
            status[b, i, j] = 3 if sr >= SIG_MIN else (2 if sf >= SIG_MIN else 1)
ADM = status == 3
print(f"(s, x) map: {NS} x {NX} points, 8 branches, {time.time() - t0:.0f} s  (settings of scripts/step07_sx_map.py)")


def step_ok(b, i, j, jp):
    """Can branch b go from (s_{i-1}, X[jp]) to (s_i, X[j])?  Same rule as the step-07 DP."""
    return ADM[b, i, j] and abs(j - jp) <= JMAX and np.abs(Q[b, i, j] - Q[b, i - 1, jp]).max() <= DQ_MAX


def step_cost(b, i, j, jp):
    return np.linalg.norm(Q[b, i, j] - Q[b, i - 1, jp]) + W_X * abs(X[j] - X[jp])


# ------------------------------------------------ DP (copy of step 07, used to check we reproduce it)
def plan_dp(b):
    INF = np.inf
    cost = np.where(ADM[b, 0] & np.isclose(X, X_START), 0.0, INF)
    back = np.zeros((NS, NX), int)
    for i in range(1, NS):
        new = np.full(NX, INF)
        for j in np.flatnonzero(ADM[b, i]):
            for jp in range(max(0, j - JMAX), min(NX, j + JMAX + 1)):
                if cost[jp] < INF and step_ok(b, i, j, jp):
                    c = cost[jp] + step_cost(b, i, j, jp)
                    if c < new[j]:
                        new[j], back[i, j] = c, jp
        cost = new
    if np.all(cost == np.inf):
        return None
    j = int(np.argmin(cost)); route = [j]
    for i in range(NS - 1, 0, -1):
        j = back[i, j]; route.append(j)
    return cost.min(), np.array(route[::-1])


# ------------------------------------------------ 5A fixed-x baseline
def why_stop(b, i, j, jp):
    """Reason the branch cannot go from (i-1, jp) to (i, j)."""
    if status[b, i, j] == 0:
        return "no root (out of reach)"
    if status[b, i, j] in (1, 2):
        return "singular (sigma_min < %.2f)" % SIG_MIN
    dq = Q[b, i, j] - Q[b, i - 1, jp]
    if np.abs(dq).max() > DQ_MAX:
        k = int(np.argmax(np.abs(dq)))
        wr = (dq + pi) % (2 * pi) - pi
        if np.abs(wr).max() <= DQ_MAX:
            return f"q{k + 1} reaches the +-pi seam (joint limit)"
        return f"continuity jump in q{k + 1}"
    return "base step too large"


def run_fixed(b, x0):
    j = int(np.argmin(np.abs(X - x0)))
    if not ADM[b, 0, j]:
        return 0.0, "not admissible at s = 0"
    for i in range(1, NS):
        if not step_ok(b, i, j, j):
            return S[i - 1], f"stops at s = {S[i]:.2f}: " + why_stop(b, i, j, j)
    return 1.0, "completes"


# label persistence along the fixed-x runs: is the nearest root (all branches, wrapped) the same label?
def nearest_is_same(b, i, j):
    d = np.full(NB, np.inf)
    for bb in range(NB):
        if status[b, i, j] and status[bb, i, j]:
            d[bb] = np.linalg.norm((Q[bb, i, j] - Q[b, i - 1, j] + pi) % (2 * pi) - pi)
    return np.argmin(d) == b


print("\n5A  fixed-x baseline: one continuation per admissible root at s = 0, base parked")
print(f"    {'branch':10s} | {'x = 0 m':58s} | x = -1 m")
for b in range(NB):
    cells = []
    for x0 in (0.0, -1.0):
        frac, msg = run_fixed(b, x0)
        cells.append(f"{msg}" if frac in (0.0, 1.0) and "stops" not in msg else f"{frac * 100:3.0f} % done, {msg}")
    print(f"    {LABELS[b]:10s} | {cells[0]:58s} | {cells[1]}")
viol = tot = 0
for x0 in (0.0, -1.0):
    j = int(np.argmin(np.abs(X - x0)))
    for b in range(NB):
        for i in range(1, NS):
            if ADM[b, i, j] and ADM[b, i - 1, j]:
                tot += 1; viol += not nearest_is_same(b, i, j)
print(f"    label persistence on these runs: nearest root has the same label in {100 * (1 - viol / max(tot, 1)):.2f} % of {tot} steps")


# ------------------------------------------------ 5B greedy continuation, x chosen locally
def run_greedy(b, rule):
    j = int(np.argmin(np.abs(X - X_START)))
    if not ADM[b, 0, j]:
        return None
    route = [j]
    for i in range(1, NS):
        cand = [jj for jj in range(max(0, j - JMAX), min(NX, j + JMAX + 1)) if step_ok(b, i, jj, j)]
        if not cand:
            # why: is there any admissible x at s_i at all, and is it within the slope bound?
            anyx = np.flatnonzero(ADM[b, i])
            if len(anyx) == 0:
                reason = "no admissible x at this s"
            elif np.min(np.abs(anyx - j)) > JMAX:
                reason = f"green band moved out of reach (nearest admissible x = {X[anyx[np.argmin(np.abs(anyx - j))]]:+.2f} m, base at {X[j]:+.2f} m)"
            else:
                reason = why_stop(b, i, anyx[np.argmin(np.abs(anyx - j))], j)
            return dict(route=np.array(route), done=S[i - 1], fail_s=S[i], reason=reason)
        if rule == "cost":
            j = min(cand, key=lambda jj: step_cost(b, i, jj, j))
        else:
            j = max(cand, key=lambda jj: SIG[0, b, i, jj])
        route.append(j)
    return dict(route=np.array(route), done=1.0, fail_s=None, reason="completes")


def route_cost(b, route):
    return sum(step_cost(b, i, route[i], route[i - 1]) for i in range(1, len(route)))


dp = {b: plan_dp(b) for b in range(NB)}
gc = {b: run_greedy(b, "cost") for b in range(NB)}
gm = {b: run_greedy(b, "margin") for b in range(NB)}
ref = {"S+ W+ E+": 11.79, "S+ W- E+": 10.31, "S+ W- E-": 10.51, "S- W+ E-": 11.04, "S- W- E+": 12.97, "S- W- E-": 12.76}
if not TAG:
    err = max(abs(dp[LABELS.index(k)][0] - v) for k, v in ref.items())
    print(f"\n5B  DP reproduces the step-07 plan costs (max difference {err:.3f}); start x = {X_START} m on every branch")
else:
    print(f"\n5B  base-speed study: |dx| <= {DX_MAX} m per path step, x grid {X[1] - X[0]:.3f} m; start x = {X_START} m")
print(f"    {'branch':10s} | {'branch-aware DP':22s} | {'greedy, min step cost':52s} | greedy, max sigma_min")


def fmt(r, b):
    if r is None:
        return "not admissible at s=0"
    if r["fail_s"] is None:
        return f"completes, cost {route_cost(b, r['route']):.2f}"
    return f"fails at s = {r['fail_s']:.2f}: {r['reason']}"


for b in range(NB):
    d = f"cost {dp[b][0]:.2f}" if dp[b] else "no schedule"
    print(f"    {LABELS[b]:10s} | {d:22s} | {fmt(gc[b], b)[:52]:52s} | {fmt(gm[b], b)}")
for name, G in (("min step cost", gc), ("max sigma_min", gm)):
    long_reasons = {LABELS[b]: G[b]["reason"] for b in range(NB) if G[b] and G[b]["fail_s"] is not None and len(fmt(G[b], b)) > 52}
    for k, v in long_reasons.items():
        print(f"      [{name}] {k}: {v}")
n_dp = sum(p is not None for p in dp.values())
n_gc = sum(1 for r in gc.values() if r and r["fail_s"] is None)
n_gm = sum(1 for r in gm.values() if r and r["fail_s"] is None)
print(f"    complete executions: DP {n_dp}/8,  greedy-cost {n_gc}/8,  greedy-margin {n_gm}/8")
b_best = min((b for b in range(NB) if dp[b]), key=lambda b: dp[b][0])


# ------------------------------------------------ 5C profiles along the executions
def profile(b, route):
    n = len(route)
    xs = X[route]
    qs = Q[b, np.arange(n), route]
    res_p, res_r = np.zeros(n), np.zeros(n)
    for i in range(n):
        T = fk_p6r(np.r_[xs[i], qs[i]])
        res_p[i] = np.linalg.norm(T[:3, 3] - POSES[i][:3, 3])
        res_r[i] = np.linalg.norm(T[:3, :3] - POSES[i][:3, :3])
    seam = (pi - np.abs(qs)).min(1)                          # distance of the closest joint to +-pi [rad]
    return dict(s=S[:n], x=xs, q=qs, seam=seam, res_p=res_p, res_r=res_r,
                sr=SIG[0, b, np.arange(n), route], sf=SIG[1, b, np.arange(n), route],
                slope=np.r_[np.nan, np.diff(xs)] / (PERIM * (S[1] - S[0])))


runs = {"branch-aware DP": profile(b_best, dp[b_best][1])}
if gc[b_best]:
    runs["greedy, min step cost"] = profile(b_best, gc[b_best]["route"])
if gm[b_best]:
    runs["greedy, max sigma_min"] = profile(b_best, gm[b_best]["route"])
print(f"\n5C  profiles on branch {LABELS[b_best]} (best DP branch)")
print(f"    {'run':22s} {'s reached':>9s} {'base travel':>11s} {'max |dx/ds|':>11s} {'min seam margin':>16s} "
      f"{'max pos res':>11s} {'min sig(J_red)':>14s} {'min sig(J)':>10s}")
for k, P in runs.items():
    print(f"    {k:22s} {P['s'][-1]:9.2f} {np.abs(np.diff(P['x'])).sum():9.2f} m {np.nanmax(np.abs(P['slope'])):11.2f} "
          f"{np.rad2deg(P['seam'].min()):12.1f} deg {P['res_p'].max():11.1e} {P['sr'].min():14.3f} {P['sf'].min():10.3f}")
print(f"    slope bound: |dx| <= {DX_MAX} m per path step of {PERIM * (S[1] - S[0]):.3f} m  ->  |dx/ds_arc| <= "
      f"{DX_MAX / (PERIM * (S[1] - S[0])):.2f}  (base speed <= {DX_MAX / (PERIM * (S[1] - S[0])):.2f} x tool speed)")

# ------------------------------------------------ figures
if os.environ.get("S5_NOFIG"):
    sys.exit(0)
GREEN, GREY = "#2ca25f", "#d9d9d6"
COL = {"branch-aware DP": "#d62728", "greedy, min step cost": "#1f77b4", "greedy, max sigma_min": "#ff7f0e"}
SS, XX = np.meshgrid(S, X, indexing="ij")

# (a) the step-07 map with DP and both greedy runs on every branch
fig, axes = plt.subplots(2, 4, figsize=(16, 7), constrained_layout=True, sharex=True, sharey=True)
for b, ax in enumerate(axes.ravel()):
    ax.scatter(SS[~ADM[b]], XX[~ADM[b]], s=5, c=GREY, linewidths=0)
    ax.scatter(SS[ADM[b]], XX[ADM[b]], s=5, c=GREEN, linewidths=0)
    for sc in S_CORNERS:
        ax.axvline(sc, color="#888", lw=0.6, ls=":")
    if dp[b]:
        ax.plot(S, X[dp[b][1]], color=COL["branch-aware DP"], lw=2.6)
    for r, k in ((gc[b], "greedy, min step cost"), (gm[b], "greedy, max sigma_min")):
        if r:
            n = len(r["route"])
            ax.plot(S[:n], X[r["route"]], color=COL[k], lw=1.6, ls="--")
            if r["fail_s"] is not None:
                ax.plot(S[n - 1], X[r["route"][-1]], "x", color=COL[k], ms=10, mew=2.5)
    ax.set_title(LABELS[b], fontsize=10)
for ax in axes[1]: ax.set_xlabel("path progress s")
for ax in axes[:, 0]: ax.set_ylabel("base position x [m]")
handles = [Line2D([], [], color=c, lw=2.4, ls="-" if k.startswith("branch") else "--", label=k) for k, c in COL.items()] + \
          [Line2D([], [], ls="", marker="x", color="k", ms=9, mew=2, label="greedy run stops"),
           Line2D([], [], ls="", marker="o", color=GREEN, label="admissible"), Line2D([], [], ls="", marker="o", color=GREY, label="not admissible")]
fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=9, bbox_to_anchor=(0.5, -0.05))
fig.suptitle(f"(s, x) map with the branch-aware plan and two greedy continuations, all from x(0) = {X_START} m", fontsize=11)
fig.savefig(OUT / f"s5_sx_map_dp_vs_greedy{TAG}.png", dpi=150, bbox_inches="tight"); plt.close(fig)

# (b) execution profiles on the best branch
fig, axs = plt.subplots(5, 1, figsize=(11, 13), sharex=True, constrained_layout=True)
for k, P in runs.items():
    c, ls = COL[k], "-" if k.startswith("branch") else "--"
    axs[0].plot(P["s"], P["x"], color=c, ls=ls, label=k)
    axs[2].plot(P["s"], np.rad2deg(P["seam"]), color=c, ls=ls)
    axs[3].semilogy(P["s"], np.maximum(P["res_p"], 1e-17), color=c, ls=ls)
    axs[4].plot(P["s"], P["sr"], color=c, ls=ls)
    axs[4].plot(P["s"], P["sf"], color=c, ls=":", lw=1)
    if P["s"][-1] < 1:
        for ax in axs:
            ax.axvline(P["s"][-1], color=c, lw=0.8, ls="--")
P = runs["branch-aware DP"]
for k in range(6):
    axs[1].plot(P["s"], np.rad2deg(P["q"][:, k]), label=f"q{k + 1}")
axs[0].set_ylabel("base x [m]"); axs[0].legend(fontsize=8)
axs[1].set_ylabel("joints, DP plan [deg]"); axs[1].legend(fontsize=8, ncol=6); axs[1].axhline(180, color="k", lw=0.6); axs[1].axhline(-180, color="k", lw=0.6)
axs[2].set_ylabel("closest joint to\n+-180 deg [deg]")
axs[3].set_ylabel("Cartesian residual\n|p_FK - p_path| [m]")
axs[4].set_ylabel("sigma_min\n(solid J_red, dotted J)"); axs[4].axhline(SIG_MIN, color="k", lw=0.6)
axs[4].set_xlabel("path progress s")
for ax in axs:
    for sc in S_CORNERS:
        ax.axvline(sc, color="#bbb", lw=0.6, ls=":")
fig.suptitle(f"Execution profiles on branch {LABELS[b_best]}: branch-aware DP vs greedy continuations", fontsize=11)
fig.savefig(OUT / f"s5_profiles{TAG}.png", dpi=140); plt.close(fig)

# (c) fixed-x baseline (5A): how far each branch gets with the base parked, and why it stops
REASON_COL = {"completes": "#2ca25f", "no root": "#9e9e9e", "seam": "#ff7f0e", "singular": "#d62728", "other": "#7b3294"}
def reason_key(msg):
    return ("completes" if msg == "completes" else "no root" if "no root" in msg else
            "seam" if "seam" in msg else "singular" if "singular" in msg else "other")
fig, axs = plt.subplots(1, 2, figsize=(13, 3.8), sharey=True, constrained_layout=True)
for ax, x0 in zip(axs, (0.0, X_START)):
    for b in range(NB):
        frac, msg = run_fixed(b, x0)
        k = reason_key(msg)
        ax.barh(b, frac, color=REASON_COL[k], height=0.7)
        ax.text(min(frac, 1) + 0.01, b, "done" if k == "completes" else f"s = {frac:.2f}", va="center", fontsize=8)
    for sc in S_CORNERS:
        ax.axvline(sc, color="#bbb", lw=0.6, ls=":")
    ax.set_xlim(0, 1.12); ax.set_title(f"base parked at x = {x0:+.1f} m"); ax.set_xlabel("path progress s reached")
axs[0].set_yticks(range(NB)); axs[0].set_yticklabels(LABELS); axs[0].invert_yaxis()
fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=REASON_COL[k]) for k in ("completes", "no root", "seam")],
           labels=["completes the path", "stops: no root (out of reach)", "stops: joint reaches the +-pi seam"],
           loc="lower center", ncol=3, fontsize=9, bbox_to_anchor=(0.5, -0.12))
fig.suptitle("Fixed-x baseline: one continuation per branch from s = 0 (dotted: rectangle corners)", fontsize=11)
fig.savefig(OUT / f"s5_fixed_x_baseline{TAG}.png", dpi=150, bbox_inches="tight"); plt.close(fig)
print(f"\nfigures: {OUT / f's5_sx_map_dp_vs_greedy{TAG}.png'}, {OUT / f's5_profiles{TAG}.png'}")
