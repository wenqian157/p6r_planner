"""Forward kinematics of the P6R (UMR x-slide + UR12e) from a standard DH table.

Standard DH row:  T = Rz(theta) Tz(d) Tx(a) Rx(alpha)
  revolute  row:  theta = q + offset,  d fixed
  prismatic row:  d     = q + offset,  theta fixed
Chain:  T_world_flange = T_BASE @ row_0 @ row_1 @ ... @ row_6 @ T_TOOL
Joint vector:  q = (x, q1, ..., q6)

DH frame placement for the slide (row 0):
  - z_0 must lie along the slide axis (world x). We put DH frame 0 at the mount height with
    x_0 = world y, y_0 = world z, z_0 = world x  (T_BASE below).
  - Row 0 has alpha_0 = -pi/2 so that z_1 = world z (joint axis 1): Rx(alpha) maps z to
    (0, -sin alpha, cos alpha), i.e. onto +y_0 only for alpha = -pi/2.
  - The common normal between the slide axis (world x) and joint axis 1 (world z) is along
    world y, so x_1 = +y. The 6R table had x along -world x at q = 0, hence a theta offset of
    +pi/2 on joint 1 to recover the same frame. Rows 2..6 and T_TOOL are the 6R ones unchanged.

Run from the repo root:
    python scripts/fk_dh_p6r.py                          -> q = 0
    python scripts/fk_dh_p6r.py 0.5 10 -60 90 -120 -90 30 -> x in m, then six angles in degrees
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_dh import DH as DH_UR, T_TOOL, dh_row, origin, Rz, Rx            # validated 6R pieces
from fk_urdf_p6r import fk as fk_urdf_p6r                                 # reference
from umr_params import MOUNT_XYZ

np.set_printoptions(precision=5, suppress=True)
pi = np.pi

# ================================ P6R DH TABLE ================================
# (type, theta_or_offset, d, a, alpha):  'P' -> d = q + d_offset, theta fixed;  'R' -> theta = q + offset
DH = [("P", 0.0,   0.0, 0.0, -pi/2)]                 # row 0: slide along z_0 = world x; Rx(-pi/2) turns z_0 into y_0 = world z
DH += [("R", pi/2, *DH_UR[0])]                       # row 1: UR row 1 with +pi/2 theta offset
DH += [("R", 0.0,  *row) for row in DH_UR[1:]]       # rows 2..6: UR rows unchanged

def R4(R):
    T = np.eye(4); T[:3, :3] = R; return T

# world -> DH frame 0: origin at mount height, columns (x0, y0, z0) = (world y, world z, world x)
T_BASE = np.eye(4)
T_BASE[:3, :3] = np.array([[0, 0, 1],
                           [1, 0, 0],
                           [0, 1, 0]], float)
T_BASE[:3, 3] = MOUNT_XYZ
# ==============================================================================


def fk(q, verbose=False):
    """T_world_flange from the P6R DH table for q = (x, q1..q6)."""
    T = T_BASE.copy()
    if verbose:
        print(f"{'frame':8s} {'origin (world)':32s} z-axis (world)")
        print(f"{'0':8s} {str(T[:3, 3]):32s} {T[:3, 2]}")
    for i, (jtype, off, d, a, alpha) in enumerate(DH):
        if jtype == "P":
            T = T @ dh_row(off, d + q[i], a, alpha)
        else:
            T = T @ dh_row(q[i] + off, d, a, alpha)
        if verbose:
            print(f"{i + 1:<8d} {str(T[:3, 3]):32s} {T[:3, 2]}")
    T = T @ T_TOOL
    if verbose:
        print(f"{'flange':8s} {str(T[:3, 3]):32s} {T[:3, 2]}")
    return T


if __name__ == "__main__":
    if len(sys.argv) == 8:
        q = np.r_[float(sys.argv[1]), np.deg2rad([float(v) for v in sys.argv[2:]])]
    else:
        q = np.zeros(7)
    print("x [m] =", q[0], "  arm q [deg] =", np.rad2deg(q[1:]), "\n")
    print("DH table (type, theta/offset, d, a, alpha):")
    for i, row in enumerate(DH):
        print(f"  row {i}: {row}")
    print()
    T_dh = fk(q, verbose=True)
    T_ref = fk_urdf_p6r(q)
    print("\nT_world_flange (DH)   =\n", T_dh)
    print("\nT_world_flange (URDF) =\n", T_ref)
    e_p = np.linalg.norm(T_dh[:3, 3] - T_ref[:3, 3])
    c = (np.trace(T_ref[:3, :3].T @ T_dh[:3, :3]) - 1) / 2
    print(f"\nposition error    = {e_p:.3e} m")
    print(f"orientation error = {np.degrees(np.arccos(np.clip(c, -1, 1))):.3e} deg")

    rng = np.random.default_rng(0)
    Q = np.c_[rng.uniform(-1.5, 1.5, 100), rng.uniform(-pi, pi, (100, 6))]
    print(f"max |T_dh - T_urdf| over 100 random q = {max(np.abs(fk(qq) - fk_urdf_p6r(qq)).max() for qq in Q):.3e}")
