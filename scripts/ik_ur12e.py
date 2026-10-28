"""Analytical IK of the UR12e (x fixed), built step by step on the validated DH table of fk_dh.py.

UR recipe (no intersecting wrist, so no Pieper split):
    step 1  theta1 from the position of O5 (wrist_2 origin)          2 solutions: shoulder left / right
    step 2  theta5 from the orientation, given theta1                2 solutions: wrist flip
    step 3  theta6 from the orientation, given theta1, theta5        unique
    step 4  theta2, theta3 from the position of O4 (planar 2R)       2 solutions: elbow up / down
    step 5  theta4 from the remaining rotation                       unique
Every step is checked here against the DH forward kinematics on random configurations.

Frames.  The IK works on T06 = pose of DH frame 6 in DH frame 0.  From a world flange pose T* and a
base position x:   T06 = (Trans(x) @ Mount @ T_BASE)^-1 @ T* @ T_TOOL^-1
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_dh import DH, T_BASE, T_TOOL, dh_row, origin, fk as fk_dh_6r
from umr_params import MOUNT_XYZ, MOUNT_RPY

pi = np.pi
np.set_printoptions(precision=4, suppress=True)

# DH constants of the UR12e (d, a, alpha per row) -> the six numbers the IK formulas use
d1, a2, a3, d4, d5, d6 = DH[0][0], DH[1][1], DH[2][1], DH[3][0], DH[4][0], DH[5][0]
wrap = lambda a: (a + pi) % (2 * pi) - pi


# ---------------------------------------------------------------- frames
def T06_from_world(T_world_flange, x):
    """Pose of DH frame 6 in DH frame 0, for the base at x (the IK input)."""
    T_world_0 = origin([x, 0, 0], [0, 0, 0]) @ origin(MOUNT_XYZ, MOUNT_RPY) @ T_BASE
    return np.linalg.inv(T_world_0) @ T_world_flange @ np.linalg.inv(T_TOOL)


def fk_rows(q):
    """T06 from the DH rows only (no base / tool transforms): what the IK must invert."""
    T = np.eye(4)
    for qi, (d, a, alpha) in zip(q, DH):
        T = T @ dh_row(qi, d, a, alpha)
    return T


# ---------------------------------------------------------------- step 1: theta1 from O5
def step1_theta1(T06):
    """O5 = origin of frame 5 = p06 - d6 * z6.  Seen from above, O5 sits on a circle of radius |p05_xy|
    around axis 1, and the arm plane that contains it is offset from axis 1 by d4.  That fixes theta1 up
    to the side of the plane O5 lies on: shoulder left / right."""
    p05 = (T06 @ np.array([0, 0, -d6, 1.0]))[:3]
    r = np.hypot(p05[0], p05[1])
    if r < d4:                                       # O5 closer to axis 1 than the plane offset: unreachable
        return []
    phi = np.arctan2(p05[1], p05[0])
    psi = np.arccos(d4 / r)
    return [wrap(phi + psi + pi / 2), wrap(phi - psi + pi / 2)]       # [shoulder 'left', shoulder 'right']


# ---------------------------------------------------------------- checks
if __name__ == "__main__":
    print("DH constants:  d1 a2 a3 d4 d5 d6 =", np.array([d1, a2, a3, d4, d5, d6]))

    # frames: T06 reconstructed from a world pose must equal the DH rows alone
    rng = np.random.default_rng(0)
    err = 0
    for _ in range(100):
        x, q = rng.uniform(-1.5, 1.5), rng.uniform(-pi, pi, 6)
        T_world = origin([x, 0, 0], [0, 0, 0]) @ origin(MOUNT_XYZ, MOUNT_RPY) @ fk_dh_6r(q)
        err = max(err, np.abs(T06_from_world(T_world, x) - fk_rows(q)).max())
    print(f"frames: max |T06_from_world - fk_rows| over 100 random (x, q) = {err:.1e}")

    # step 1: the true theta1 must be one of the two candidates
    worst, which = 0, []
    for _ in range(1000):
        q = rng.uniform(-pi, pi, 6)
        cands = step1_theta1(fk_rows(q))
        errs = [abs(wrap(c - q[0])) for c in cands]
        worst = max(worst, min(errs))
        which.append(int(np.argmin(errs)))
    print(f"step 1: max |theta1_true - nearest candidate| over 1000 random q = {worst:.1e} rad")
    print(f"        true theta1 was candidate 0 ('left') in {which.count(0)} cases, candidate 1 ('right') in {which.count(1)}")

    # one configuration, printed
    q = np.deg2rad([10, -60, 90, -120, -90, 30])
    T06 = fk_rows(q)
    p05 = (T06 @ np.array([0, 0, -d6, 1.0]))[:3]
    print(f"\nexample q = {np.rad2deg(q)} deg")
    print(f"  p06 (flange in frame 0) = {T06[:3, 3]}")
    print(f"  p05 (O5 in frame 0)     = {p05}     |p05_xy| = {np.hypot(p05[0], p05[1]):.4f} m  (must be >= d4 = {d4})")
    print(f"  theta1 candidates [deg] = {np.rad2deg(step1_theta1(T06))}   (true: {np.rad2deg(q[0]):.1f})")
