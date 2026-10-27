"""Forward kinematics of the P6R (UMR prismatic x-slide + UR12e) from the URDF numbers.

Same loop as fk_urdf.py, with one branch for the joint type:
    T = T @ Origin(xyz, rpy) @ Motion(type, axis, q_i)
    Motion = rotation about axis (revolute) | translation along axis (prismatic) | identity (fixed)

Chain:  world --(prismatic, x)--> umr_base --(fixed mount)--> ur base_link --(UR12e chain)--> flange
Joint vector:  q = (x, q1, ..., q6)   [m, rad x 6]

Run from the repo root:
    python scripts/fk_urdf_p6r.py                          -> q = 0
    python scripts/fk_urdf_p6r.py 0.5 10 -60 90 -120 -90 30 -> x in m, then six angles in degrees
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_urdf import CHAIN as UR_CHAIN, origin, rot_axis, fk as fk_ur6   # the validated 6R model
from umr_params import MOUNT_XYZ, MOUNT_RPY

np.set_printoptions(precision=5, suppress=True)
pi = np.pi

# ---------------------------------------------------------------- the P6R chain
# (name, xyz, rpy, type, axis)   type: 'P' prismatic, 'R' revolute, None fixed
CHAIN = [
    ("0 base_x  world -> umr_base",   [0, 0, 0],      [0, 0, 0],  "P",  [1, 0, 0]),
    ("mount    umr_base -> base_link", list(MOUNT_XYZ), list(MOUNT_RPY), None, None),
] + [(name, xyz, rpy, "R" if axis is not None else None, axis) for name, xyz, rpy, axis in UR_CHAIN]


def trans_axis(axis, q):
    T = np.eye(4)
    T[:3, 3] = q * np.asarray(axis, float) / np.linalg.norm(axis)
    return T


def fk(q, verbose=False):
    """T_world_flange for q = (x, q1..q6)."""
    T, k = np.eye(4), 0
    for name, xyz, rpy, jtype, axis in CHAIN:
        T = T @ origin(xyz, rpy)
        if jtype == "R":
            T = T @ rot_axis(axis, q[k]); k += 1
        elif jtype == "P":
            T = T @ trans_axis(axis, q[k]); k += 1
        if verbose:
            print(f"after {name:32s} position = {T[:3, 3]}")
    assert k == len(q) == 7
    return T


if __name__ == "__main__":
    if len(sys.argv) == 8:
        q = np.r_[float(sys.argv[1]), np.deg2rad([float(v) for v in sys.argv[2:]])]
    else:
        q = np.zeros(7)
    print("x [m] =", q[0], "  arm q [deg] =", np.rad2deg(q[1:]), "\n")
    T = fk(q, verbose=True)
    print("\nT_world_flange =\n", T)

    # consistency with the 6R model: the P6R flange is the UR flange moved by the mount and the slide
    T_expect = origin([q[0], 0, 0], [0, 0, 0]) @ origin(MOUNT_XYZ, MOUNT_RPY) @ fk_ur6(q[1:])
    print(f"\nvs 6R model shifted by slide + mount: max |diff| = {np.abs(T - T_expect).max():.2e}")
