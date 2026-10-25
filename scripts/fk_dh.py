"""Forward kinematics of the UR12e from a standard DH table (lecture 1, slide 18).

One DH row per joint:
    T_{i-1,i} = Rz(theta_i) @ Tz(d_i) @ Tx(a_i) @ Rx(alpha_i),    theta_i = q_i + OFFSET[i]
Full chain:
    T_world_flange = T_BASE @ T_01 @ T_12 @ ... @ T_56 @ T_TOOL
T_BASE takes the world frame to DH frame 0, T_TOOL takes DH frame 6 to the flange.

Run from the repo root:
    python scripts/fk_dh.py                       -> q = 0, and a comparison with fk_urdf
    python scripts/fk_dh.py 10 -60 90 -120 -90 30 -> q in degrees
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_urdf import fk as fk_urdf, Rx, Ry, Rz, origin   # reference model and helpers

np.set_printoptions(precision=5, suppress=True)
pi = np.pi

# ================================ DH TABLE (fill in) ================================
# Row i:  (d_i, a_i, alpha_i)   [m, m, rad]
# Vendor sheet for reference (their frame choice, not necessarily yours):
#   d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
#   a = [0, -0.6127, -0.57155, 0, 0, 0]
#   alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
DH = [
    (0.1807,  0.0,      pi/2),   # 1: axes 1,2 intersect at z = 0.1807
    (0.0,    -0.6127,   0.0),    # 2: upper arm, axes 2,3 parallel
    (0.0,    -0.57155,  0.0),    # 3: forearm,   axes 3,4 parallel
    (0.17415, 0.0,      pi/2),   # 4: wrist side offset, axis 5 perpendicular to 4
    (0.11985, 0.0,     -pi/2),   # 5: wrist offset,      axis 6 perpendicular to 5
    (0.11655, 0.0,      0.0),    # 6: flange offset
]
OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]        # theta_i = q_i + OFFSET[i]; DH zero = URDF zero
T_BASE = origin([0, 0, 0], [0, 0, pi])         # world -> DH frame 0 = URDF base_link_inertia (yaw pi)
T_TOOL = origin([0, 0, 0], [0, -pi/2, -pi/2])  # DH frame 6 = URDF wrist_3_link; this is wrist_3 -> flange
# ====================================================================================


def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T

def Tx(a):
    T = np.eye(4); T[0, 3] = a; return T

def R4(R):
    T = np.eye(4); T[:3, :3] = R; return T

def dh_row(theta, d, a, alpha):
    """Standard DH step: four ordered motions."""
    return R4(Rz(theta)) @ Tz(d) @ Tx(a) @ R4(Rx(alpha))


def fk(q, verbose=False):
    """T_world_flange from the DH table for the six joint angles q [rad]."""
    T = T_BASE.copy()
    if verbose:
        print(f"{'frame':8s} {'origin (world)':30s} z-axis (world)")
        print(f"{'0':8s} {str(T[:3, 3]):30s} {T[:3, 2]}")
    for i, (d, a, alpha) in enumerate(DH):
        T = T @ dh_row(q[i] + OFFSET[i], d, a, alpha)
        if verbose:
            print(f"{i + 1:<8d} {str(T[:3, 3]):30s} {T[:3, 2]}")
    T = T @ T_TOOL
    if verbose:
        print(f"{'flange':8s} {str(T[:3, 3]):30s} {T[:3, 2]}")
    return T


if __name__ == "__main__":
    q = np.deg2rad([float(v) for v in sys.argv[1:]]) if len(sys.argv) == 7 else np.zeros(6)
    print("q [deg] =", np.rad2deg(q), "\n")
    T_dh = fk(q, verbose=True)
    T_ref = fk_urdf(q)

    print("\nT_world_flange (DH) =");   print(T_dh)
    print("\nT_world_flange (URDF) ="); print(T_ref)

    e_p = np.linalg.norm(T_dh[:3, 3] - T_ref[:3, 3])
    c = (np.trace(T_ref[:3, :3].T @ T_dh[:3, :3]) - 1) / 2
    e_R = np.degrees(np.arccos(np.clip(c, -1, 1)))
    print(f"\nposition error    = {e_p:.3e} m")
    print(f"orientation error = {e_R:.3e} deg")
