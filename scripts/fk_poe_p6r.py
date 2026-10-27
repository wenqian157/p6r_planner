"""Forward kinematics of the P6R (UMR x-slide + UR12e) by the Product of Exponentials.

    T_world_flange(q) = exp([S0] x) @ exp([S1] q1) @ ... @ exp([S6] q6) @ M
S0 is the prismatic screw of the slide:  w = 0,  v = unit direction of travel (world x).
S1..S6 are the UR12e screws of fk_poe.py, with their axis points lifted by the mount offset
(a direction does not change under translation, a point on the axis does).
M is the 6R home pose moved by the mount offset.  Home configuration: x = 0, arm q = 0.

Run from the repo root:
    python scripts/fk_poe_p6r.py                          -> q = 0
    python scripts/fk_poe_p6r.py 0.5 10 -60 90 -120 -90 30 -> x in m, then six angles in degrees
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_poe import AXES as UR_AXES, M as M_UR, screw, skew       # validated 6R pieces
from fk_urdf_p6r import fk as fk_urdf_p6r                        # reference
from umr_params import MOUNT_XYZ

np.set_printoptions(precision=5, suppress=True)
pi = np.pi
mount = np.asarray(MOUNT_XYZ, float)

# ================================ SCREWS (home configuration) ================================
SCREWS = [(np.zeros(3), np.array([1.0, 0, 0]))]                       # S0: prismatic along world x
SCREWS += [screw(w, np.asarray(r, float) + mount) for _, w, r in UR_AXES]   # S1..S6 lifted by the mount
NAMES = ["0 base_x"] + [name for name, _, _ in UR_AXES]
M = M_UR.copy()
M[:3, 3] += mount
# ============================================================================================


def exp_twist(w, v, q):
    """exp([S] q): rotation screw (|w| = 1) or pure translation (w = 0)."""
    T = np.eye(4)
    if np.allclose(w, 0):
        T[:3, 3] = q * v
        return T
    K = skew(w)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    T[:3, 3] = (np.eye(3) * q + (1 - np.cos(q)) * K + (q - np.sin(q)) * (K @ K)) @ v
    return T


def fk(q, verbose=False):
    """T_world_flange by the product of exponentials for q = (x, q1..q6)."""
    T = np.eye(4)
    for i, (w, v) in enumerate(SCREWS):
        T = T @ exp_twist(w, v, q[i])
        if verbose:
            print(f"after exp(S{i} q{i}):  moved flange = {(T @ M)[:3, 3]}")
    return T @ M


if __name__ == "__main__":
    if len(sys.argv) == 8:
        q = np.r_[float(sys.argv[1]), np.deg2rad([float(v) for v in sys.argv[2:]])]
    else:
        q = np.zeros(7)
    print("=== screws S_i = (w_i, v_i), home configuration ===")
    for name, (w, v) in zip(NAMES, SCREWS):
        print(f"{name:18s} w = {w}   v = {v}")
    print("\nM (home pose of the flange) =\n", M)

    print("\nx [m] =", q[0], "  arm q [deg] =", np.rad2deg(q[1:]), "\n")
    T_poe = fk(q, verbose=True)
    T_ref = fk_urdf_p6r(q)
    print("\nT_world_flange (PoE)  =\n", T_poe)
    print("\nT_world_flange (URDF) =\n", T_ref)
    e_p = np.linalg.norm(T_poe[:3, 3] - T_ref[:3, 3])
    c = (np.trace(T_ref[:3, :3].T @ T_poe[:3, :3]) - 1) / 2
    print(f"\nposition error    = {e_p:.3e} m")
    print(f"orientation error = {np.degrees(np.arccos(np.clip(c, -1, 1))):.3e} deg")

    rng = np.random.default_rng(0)
    Q = np.c_[rng.uniform(-1.5, 1.5, 100), rng.uniform(-pi, pi, (100, 6))]
    print(f"max |T_poe - T_urdf| over 100 random q = {max(np.abs(fk(qq) - fk_urdf_p6r(qq)).max() for qq in Q):.3e}")
