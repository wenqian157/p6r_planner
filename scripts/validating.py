"""Validate the three P6R forward-kinematics models against each other.

Input  : q = (x [m], q1..q6 [deg])   hard-coded below or given on the command line
Methods: URDF chain (fk_urdf_p6r), DH table (fk_dh_p6r), product of exponentials (fk_poe_p6r)
Output : the three T_world_flange matrices, pairwise position / orientation errors, a random-sample
         sweep, printed and written to results/fk_validation.md (Markdown, ready for the report).

Run from the repo root:
    python scripts/validating.py
    python scripts/validating.py 0.5 10 -60 90 -120 -90 30
"""
import sys
from pathlib import Path
from datetime import date
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fk_urdf_p6r import fk as fk_urdf
from fk_dh_p6r import fk as fk_dh
from fk_poe_p6r import fk as fk_poe

pi = np.pi
np.set_printoptions(precision=6, suppress=True)

# ------------------------------------------------ input (x in m, arm in deg)
x_m = 0.5
q_deg = [10, -60, 90, -120, -90, 30]
N_RANDOM = 1000
OUT = HERE.parent / "results" / "fk_validation.md"
# -------------------------------------------------------------------------
if len(sys.argv) == 8:
    x_m, q_deg = float(sys.argv[1]), [float(v) for v in sys.argv[2:]]
q = np.r_[x_m, np.deg2rad(q_deg)]

METHODS = [("URDF chain", fk_urdf), ("DH table", fk_dh), ("PoE", fk_poe)]


def pose_diff(Ta, Tb):
    """(position error [m], orientation error [deg]) between two homogeneous transforms."""
    e_p = np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])
    dR = Ta[:3, :3].T @ Tb[:3, :3]                       # relative rotation
    s = 0.5 * np.linalg.norm([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])   # sin(angle)
    c = 0.5 * (np.trace(dR) - 1)                                                                # cos(angle)
    return e_p, np.degrees(np.arctan2(s, c))             # atan2 is exact for small angles, arccos is not


def mat_md(T):
    return "\n".join("| " + " | ".join(f"{v: .6f}" for v in row) + " |" for row in T)


lines = [f"# FK validation, P6R (UMR x-slide + UR12e)", "",
         f"Date: {date.today().isoformat()}  ",
         f"Input: x = {x_m} m, q1..q6 = {q_deg} deg", ""]

# 1. the three results
Ts = {}
for name, f in METHODS:
    Ts[name] = f(q)
    lines += [f"## T_world_flange, {name}", "", "| | | | |", "|---|---|---|---|", mat_md(Ts[name]), ""]

# 2. pairwise errors
lines += ["## Pairwise errors at the input q", "",
          "| pair | position [m] | orientation [deg] |", "|---|---|---|"]
pairs = [("URDF chain", "DH table"), ("URDF chain", "PoE"), ("DH table", "PoE")]
for a, b in pairs:
    e_p, e_r = pose_diff(Ts[a], Ts[b])
    lines.append(f"| {a} vs {b} | {e_p:.2e} | {e_r:.2e} |")
lines.append("")

# 3. random sweep
rng = np.random.default_rng(0)
Q = np.c_[rng.uniform(-1.5, 1.5, N_RANDOM), rng.uniform(-pi, pi, (N_RANDOM, 6))]
worst = {p: (0.0, 0.0) for p in pairs}
for qq in Q:
    T = {name: f(qq) for name, f in METHODS}
    for a, b in pairs:
        e_p, e_r = pose_diff(T[a], T[b])
        worst[(a, b)] = (max(worst[(a, b)][0], e_p), max(worst[(a, b)][1], e_r))
lines += [f"## Worst errors over {N_RANDOM} random q  (x in [-1.5, 1.5] m, angles in [-pi, pi])", "",
          "| pair | max position [m] | max orientation [deg] |", "|---|---|---|"]
for a, b in pairs:
    lines.append(f"| {a} vs {b} | {worst[(a, b)][0]:.2e} | {worst[(a, b)][1]:.2e} |")
tol_p, tol_r = 1e-9, 1e-6
ok = all(wp < tol_p and wr < tol_r for wp, wr in worst.values())
lines += ["", f"Verdict: {'the three models agree' if ok else 'MISMATCH'} "
          f"(tolerance {tol_p:.0e} m, {tol_r:.0e} deg).", ""]

text = "\n".join(lines)
print(text)
OUT.parent.mkdir(exist_ok=True)
OUT.write_text(text, encoding="utf-8")
print(f"written to {OUT.relative_to(HERE.parent)}")
