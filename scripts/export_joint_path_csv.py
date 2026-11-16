"""Export the planned joint path of the best branch as CSV for the course visualiser (module 5).

Format: 7 columns, one row per path sample: base_x [m], q1..q6 [rad], header = URDF joint names.
The rows pair one-to-one with results/paths/workspace_path_<tag>_flange.csv (same 221 samples of s).

MODEL chooses whose kinematics the joint values are for (they must match the URDF loaded in the viewer):
    "urdf"    umr_urdf/umr_ur12e.urdf   mount (0, 0, 0.41) m, yaw pi       <- the 7-joint URDF in the repo
    "report"  scripts/umr_params.py     mount (0, 0, 0.42) m, yaw 0        <- the model of the report figures
The same world path is planned for the chosen model: (s, x) map, dynamic programming from X_START on every
branch, best branch kept.  The staircase x(s) of the grid search is then smoothed with a moving average and
the arm is re-solved analytically at the smoothed x (still an exact IK root at every sample); the result is
checked for admissibility, joint continuity and by forward kinematics on the chosen model.

Run from the repo root:
    python scripts/export_joint_path_csv.py                  horizontal rectangle, model urdf
    python scripts/export_joint_path_csv.py 30 20            tilted 30 deg about x, 20 deg about y
    python scripts/export_joint_path_csv.py 30 20 report     same, report model
Output: results/paths/joint_path_<tag>_<model>.csv
"""
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
from ik_ur12e import ik
from fk_dh import T_BASE, T_TOOL
from jacobian_p6r import jacobian_reduced
from fk_urdf_p6r import fk as fk_report
from umr_params import MOUNT_XYZ
from p6r.urdf import UrdfChain

smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]

TILT_X, TILT_Y, MODEL = 0.0, 0.0, "urdf"
args = sys.argv[1:]
if len(args) >= 2:
    TILT_X, TILT_Y = float(args[0]), float(args[1])
if len(args) == 3:
    MODEL = args[2]

# ------------------------------------------------ path (same as export_path_csv.py) and planner settings
PIVOT = np.array([0.0, 0.25, 0.9])
UV = np.array([[-0.8, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6]])
R_FLAT = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
N_SAMPLES = 221
S = np.linspace(0, 1, N_SAMPLES)
X = np.arange(-1.0, 1.0 + 1e-9, 0.05)
SIG_MIN, DX_MAX, DQ_MAX, W_X, X_START = 0.03, 0.05, 0.5, 2.0, -1.0
SMOOTH = 15                                              # moving-average window on x(s) [samples], odd
LABELS = ["S+ W+ E+", "S+ W+ E-", "S+ W- E+", "S+ W- E-", "S- W+ E+", "S- W+ E-", "S- W- E+", "S- W- E-"]
# -----------------------------------------------------------------------------
cx, sx_, cy, sy_ = (f(np.deg2rad(a)) for a in (TILT_X, TILT_Y) for f in (np.cos, np.sin))
R_TILT = np.array([[1, 0, 0], [0, cx, -sx_], [0, sx_, cx]]) @ np.array([[cy, 0, -sy_], [0, 1, 0], [sy_, 0, cy]])
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


# ------------------------------------------------ the chosen model: world -> DH frame 0, FK, joint names
if MODEL == "urdf":
    chain = UrdfChain(ROOT / "umr_urdf" / "umr_ur12e.urdf", "world", "flange")
    T_W0_AT_0 = chain.frames(np.zeros(7))["base_link_inertia"]          # includes mount and both yaws
    fk_model = chain.fk
    names = chain.names
    mount_txt = "umr_urdf/umr_ur12e.urdf (mount z = 0.41 m, yaw pi)"
else:
    T_W0_AT_0 = np.eye(4)
    T_W0_AT_0[:3, 3] = MOUNT_XYZ
    T_W0_AT_0 = T_W0_AT_0 @ T_BASE
    fk_model = fk_report
    names = ["base_x_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    mount_txt = "scripts/umr_params.py (mount z = 0.42 m, yaw 0)"
T_TOOL_INV = np.linalg.inv(T_TOOL)


def ik_model(T_world_flange, x):
    T_w0 = T_W0_AT_0.copy()
    T_w0[0, 3] += x
    return ik(np.linalg.inv(T_w0) @ T_world_flange @ T_TOOL_INV)


def root(b_label, s, x):
    """(q, sigma_min(J_red)) of one branch at (s, x), or None."""
    for sol in ik_model(pose(s), x):
        if sol["label"] == b_label:
            return sol["q"], smin(jacobian_reduced(np.r_[0.0, sol["q"]]))    # sigma_min does not depend on the mount
    return None


# ------------------------------------------------ 1. map and plan on the grid
NB, NS, NX = len(LABELS), len(S), len(X)
adm = np.zeros((NB, NS, NX), bool)
Q = np.full((NB, NS, NX, 6), np.nan)
for i, s in enumerate(S):
    T = pose(s)
    for j, x in enumerate(X):
        for sol in ik_model(T, x):
            b = LABELS.index(sol["label"])
            Q[b, i, j] = sol["q"]
            adm[b, i, j] = smin(jacobian_reduced(np.r_[0.0, sol["q"]])) >= SIG_MIN


def plan(b):
    INF = np.inf
    cost = np.where(adm[b, 0] & np.isclose(X, X_START), 0.0, INF)
    back = np.zeros((NS, NX), int)
    jmax = int(round(DX_MAX / (X[1] - X[0])))
    for i in range(1, NS):
        new = np.full(NX, INF)
        for j in np.flatnonzero(adm[b, i]):
            for jp in range(max(0, j - jmax), min(NX, j + jmax + 1)):
                if cost[jp] == INF:
                    continue
                dq = Q[b, i, j] - Q[b, i - 1, jp]
                if np.abs(dq).max() > DQ_MAX:
                    continue
                c = cost[jp] + np.linalg.norm(dq) + W_X * abs(X[j] - X[jp])
                if c < new[j]:
                    new[j], back[i, j] = c, jp
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
assert feasible, "no branch has a complete schedule from the start"
b_best = min(feasible, key=lambda b: feasible[b][0])
cost_grid, route = feasible[b_best]
x_stair = X[route]

# ------------------------------------------------ 2. smooth x(s) and re-solve the arm analytically
pad = SMOOTH // 2
x_smooth = np.convolve(np.r_[np.full(pad, x_stair[0]), x_stair, np.full(pad, x_stair[-1])], np.ones(SMOOTH) / SMOOTH, mode="valid")


def solve_along(xs):
    qs, sig = [], []
    for s, x in zip(S, xs):
        r = root(LABELS[b_best], s, x)
        if r is None or r[1] < SIG_MIN:
            return None
        qs.append(r[0]); sig.append(r[1])
    qs = np.array(qs)
    return (qs, np.array(sig)) if np.abs(np.diff(qs, axis=0)).max() <= DQ_MAX else None


res, used = solve_along(x_smooth), "smoothed"
if res is None:
    res, used, x_smooth = solve_along(x_stair), "staircase (smoothing left the admissible region)", x_stair
qs, sig = res
path = np.c_[x_smooth, qs]

# ------------------------------------------------ 3. checks and output
e_p = e_r = 0.0
for s, q7 in zip(S, path):
    T, Tt = fk_model(q7), pose(s)
    e_p = max(e_p, np.linalg.norm(T[:3, 3] - Tt[:3, 3]))
    dR = T[:3, :3].T @ Tt[:3, :3]
    e_r = max(e_r, np.degrees(np.arctan2(0.5 * np.linalg.norm([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]), 0.5 * (np.trace(dR) - 1))))
tag = ("".join(f"{n}{a:g}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a) or "flat")
out = ROOT / "results" / "paths" / f"joint_path_{tag}_{MODEL}.csv"
out.parent.mkdir(parents=True, exist_ok=True)
np.savetxt(out, path, delimiter=",", header=",".join(names), comments="", fmt="%.6f")

d = np.diff(path, axis=0)
print(f"written {out.relative_to(ROOT)}: {len(path)} rows x 7 columns ({', '.join(names)})")
print(f"  model: {mount_txt}")
print(f"  path: rectangle tilted {TILT_X:g} deg about x and {TILT_Y:g} deg about y; pairs with workspace_path_{tag}_flange.csv")
print(f"  feasible branches from x = {X_START} m: " + ", ".join(f"{LABELS[b]} ({feasible[b][0]:.2f})" for b in sorted(feasible, key=lambda b: feasible[b][0])))
print(f"  exported branch: {LABELS[b_best]}, base schedule {used}, x from {path[0, 0]:+.3f} to {path[-1, 0]:+.3f} m, base travel {np.abs(d[:, 0]).sum():.2f} m")
print(f"  largest step between rows: base {np.abs(d[:, 0]).max() * 1000:.1f} mm, joints {np.rad2deg(np.abs(d[:, 1:]).max()):.2f} deg")
print(f"  joint ranges [deg]: " + ", ".join(f"q{k} {np.rad2deg(path[:, k].min()):.0f}..{np.rad2deg(path[:, k].max()):.0f}" for k in range(1, 7)))
print(f"  min sigma_min(J_red) = {sig.min():.3f}")
print(f"  FK check on this model against the workspace path: max position error {e_p:.1e} m, max orientation error {e_r:.1e} deg")
