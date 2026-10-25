"""Step 2: fill in the standard DH table of the UR12e and check it against the URDF reference.

Standard DH row:   T_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i),   theta_i = q_i + offset_i
Full chain:        T_world_flange = T_BASE * T_1 * ... * T_6 * T_TOOL

Fill in DH, THETA_OFFSET, T_BASE, T_TOOL below, then run from the repo root:
    python scripts/step02_dh_check.py

Two checks are printed:
  (a) frame-by-frame: after k DH rows, the DH frame's z-axis must be parallel to URDF joint
      axis k+1 and its origin must lie on that axis line (at q = 0). Tells you WHICH row is wrong.
  (b) end-to-end: flange pose error vs the URDF FK at random q within +/- pi.
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from p6r.urdf import UrdfChain
from p6r.se3 import Rz, Ry, Rx, T_from, R_of, p_of, pose_error, rpy_to_R

np.set_printoptions(precision=5, suppress=True)
URDF = Path(__file__).resolve().parents[1] / "ur12e" / "ur12e.urdf"
pi = np.pi

# ============================ YOUR DH MODEL (edit here) ============================
# One row per joint:  (d, a, alpha)   [m, m, rad]
DH = [
    (0.1807,  0.0,      pi/2),   # 1
    (0.0,    -0.6127,   0.0),    # 2
    (0.0,    -0.57155,  0.0),    # 3
    (0.17415, 0.0,      pi/2),   # 4
    (0.11985, 0.0,     -pi/2),   # 5
    (0.11655, 0.0,      0.0),    # 6
]
THETA_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]          # theta_i = q_i + offset_i
T_BASE = T_from(R=rpy_to_R(0, 0, pi), p=[0, 0, 0])     # world -> DH frame 0 (= base_link_inertia)
T_TOOL = T_from(R=rpy_to_R(0, -pi/2, -pi/2), p=[0, 0, 0])  # DH frame 6 (= wrist_3_link) -> flange
# ==================================================================================


def dh_row(theta, d, a, alpha):
    return T_from(R=Rz(theta)) @ T_from(p=[0, 0, d]) @ T_from(p=[a, 0, 0]) @ T_from(R=Rx(alpha))


def fk_dh(q, upto=6):
    """T_world_{DH frame upto} (upto=6 plus T_TOOL gives the flange)."""
    T = T_BASE.copy()
    for i in range(upto):
        d, a, alpha = DH[i]
        T = T @ dh_row(q[i] + THETA_OFFSET[i], d, a, alpha)
    if upto == 6:
        T = T @ T_TOOL
    return T


def dist_point_line(p, r, w):
    """Distance from point p to the line through r with unit direction w."""
    return np.linalg.norm(np.cross(p - r, w))


arm = UrdfChain(URDF, root="world", tip="flange")
q0 = np.zeros(6)
F0 = arm.frames(q0)

print("=== (a) frame-by-frame check at q = 0 ===")
print(f"{'after row':10s} {'z_k parallel to axis k+1?':28s} {'origin on axis k+1? [m]':26s}")
ok_all = True
for k in range(6):
    Tk = fk_dh(q0, upto=k)                      # DH frame k (k=0 is the base frame)
    z_k, o_k = R_of(Tk)[:, 2], p_of(Tk)
    j = arm.moving[k]                           # URDF joint k+1
    Tc = F0[j.child]
    w, r = R_of(Tc) @ j.axis, p_of(Tc)
    par = np.linalg.norm(np.cross(z_k, w))      # 0 if parallel
    dist = dist_point_line(o_k, r, w)           # 0 if on the line
    flag = "ok" if par < 1e-8 and dist < 1e-8 else "MISMATCH"   # URDF rounds pi/2 to 1e-10
    ok_all &= flag == "ok"
    print(f"{k:<10d} {par:<28.2e} {dist:<26.2e} {flag}")

print("\n=== (b) end-to-end check, random q in [-pi, pi] ===")
rng = np.random.default_rng(0)
worst_p = worst_R = 0.0
for _ in range(200):
    q = rng.uniform(-pi, pi, 6)
    e_p, e_R = pose_error(fk_dh(q), arm.fk(q))
    worst_p, worst_R = max(worst_p, e_p), max(worst_R, e_R)
print(f"max position error:    {worst_p:.3e} m")
print(f"max orientation error: {np.rad2deg(worst_R):.3e} deg")

print("\n=== flange pose at q = 0: DH (top) vs URDF (bottom) ===")
print(fk_dh(q0))
print(arm.fk(q0))
print("\nRESULT:", "DH model matches the URDF" if ok_all and worst_p < 1e-8 and worst_R < np.deg2rad(1e-4)
      else "not yet - see MISMATCH rows above")
