"""Analytical IK of the UR12e (x fixed), built on the validated DH table of fk_dh.py.

Three groups, following the plane-split decoupling (no intersecting wrist, so no Pieper split):
    group 1  shoulder  q1        from the position of O5 (wrist_2 origin)        2 branches: S+ / S-
    group 2  wrist     q5, q6    from the orientation R06, given q1              2 branches: W+ / W-
    group 3  planar    q2,q3,q4  planar 3R in the arm plane, after stripping     2 branches: E+ / E-
                                 joints 1, 5, 6 off the pose
Up to 2 x 2 x 2 = 8 joint vectors per pose.  Joint limits are +/- pi on every joint (project convention):
every angle is wrapped to (-pi, pi], so each geometric branch yields exactly one candidate.

Frames.  The IK works on T06 = pose of DH frame 6 (flange) in DH frame 0 (base_link_inertia).  From a
world flange pose T* and a base position x:   T06 = (Trans(x) @ Mount @ T_BASE)^-1 @ T* @ T_TOOL^-1

Formulas follow the standard UR closed form (Hawkins 2013, ur_kinematics), written for the vendor DH
table d = (d1, 0, 0, d4, d5, d6), a = (0, a2, a3, 0, 0, 0), alpha = (pi/2, 0, 0, pi/2, -pi/2, 0).
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_dh import DH, T_BASE, T_TOOL, dh_row, origin, fk as fk_dh_6r
from umr_params import MOUNT_XYZ, MOUNT_RPY

pi = np.pi
np.set_printoptions(precision=4, suppress=True)

d1, a2, a3, d4, d5, d6 = DH[0][0], DH[1][1], DH[2][1], DH[3][0], DH[4][0], DH[5][0]
wrap = lambda a: (a + pi) % (2 * pi) - pi
EPS = 1e-9


# ---------------------------------------------------------------- frames
def T06_from_world(T_world_flange, x):
    """Pose of DH frame 6 in DH frame 0, for the base at x (the IK input)."""
    T_world_0 = origin([x, 0, 0], [0, 0, 0]) @ origin(MOUNT_XYZ, MOUNT_RPY) @ T_BASE
    return np.linalg.inv(T_world_0) @ T_world_flange @ np.linalg.inv(T_TOOL)


def fk_rows(q):
    """T06 from the DH rows only (no base / tool transforms): what the IK inverts."""
    T = np.eye(4)
    for qi, (d, a, alpha) in zip(q, DH):
        T = T @ dh_row(qi, d, a, alpha)
    return T


def row(i, theta):
    d, a, alpha = DH[i]
    return dh_row(theta, d, a, alpha)


# ---------------------------------------------------------------- group 1: shoulder
def group1_shoulder(T06):
    """q1 from O5 = p06 - d6 z6.  Seen from above, O5 lies at radius r from axis 1 while the arm plane
    that contains it is offset from axis 1 by d4: q1 = phi +/- acos(d4 / r) + pi/2.  Fails when r < d4,
    and is singular when r = d4 (shoulder family, rho = 0)."""
    p05 = (T06 @ np.array([0, 0, -d6, 1.0]))[:3]
    r = np.hypot(p05[0], p05[1])
    if r < d4 - EPS:
        return []
    phi = np.arctan2(p05[1], p05[0])
    psi = np.arccos(np.clip(d4 / r, -1, 1))
    return [("S+", wrap(phi + psi + pi / 2)), ("S-", wrap(phi - psi + pi / 2))]


# ---------------------------------------------------------------- group 2: wrist
def group2_wrist(T06, q1):
    """q5, q6 from the orientation, given q1.  The arm-plane normal is n = (sin q1, -cos q1, 0) in
    frame 0.  cos q5 = (p06 . n - d4) / d6 is the tool axis component along n; q6 follows from the
    components of x6, y6 along n.  Singular when sin q5 = 0 (wrist family): q6 is then free, set to 0."""
    s1, c1 = np.sin(q1), np.cos(q1)
    c5 = (T06[0, 3] * s1 - T06[1, 3] * c1 - d4) / d6
    if abs(c5) > 1 + EPS:
        return []
    c5 = np.clip(c5, -1, 1)
    out = []
    for lbl, q5 in (("W+", np.arccos(c5)), ("W-", -np.arccos(c5))):
        s5 = np.sin(q5)
        if abs(s5) < EPS:
            out.append((lbl, q5, 0.0, True))                 # wrist singular: q6 undefined
            continue
        sg = np.sign(s5)
        q6 = np.arctan2(sg * (-T06[0, 1] * s1 + T06[1, 1] * c1), sg * (T06[0, 0] * s1 - T06[1, 0] * c1))
        out.append((lbl, q5, wrap(q6), False))
    return out


# ---------------------------------------------------------------- group 3: planar arm
def group3_planar(T06, q1, q5, q6):
    """q2, q3, q4 from T14 = T01^-1 T06 T56^-1 T45^-1: a planar 3R in the arm plane.  The point
    p13 = O4 - O1 expressed in frame 1 (x, y in the plane) must be reached by links a2, a3 (2R: elbow
    E+ / E-), then q4 is the remaining in-plane rotation.  Singular when sin q3 = 0 (elbow family)."""
    T14 = np.linalg.inv(row(0, q1)) @ T06 @ np.linalg.inv(row(5, q6)) @ np.linalg.inv(row(4, q5))
    p13 = (T14 @ np.array([0, -d4, 0, 1.0]))[:3]
    L2 = p13[0] ** 2 + p13[1] ** 2
    c3 = (L2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
    if abs(c3) > 1 + EPS:
        return []
    c3 = np.clip(c3, -1, 1)
    out = []
    for lbl, q3 in (("E+", np.arccos(c3)), ("E-", -np.arccos(c3))):
        q2 = -np.arctan2(p13[1], -p13[0]) + np.arcsin(np.clip(a3 * np.sin(q3) / np.sqrt(L2), -1, 1))
        T34 = np.linalg.inv(row(2, q3)) @ np.linalg.inv(row(1, q2)) @ T14
        q4 = np.arctan2(T34[1, 0], T34[0, 0])
        out.append((lbl, wrap(q2), wrap(q3), wrap(q4)))
    return out


# ---------------------------------------------------------------- assembled solver
def ik(T06):
    """All IK branches of the UR12e for T06.  Returns a list of dicts:
         q      joint vector (6,) wrapped to (-pi, pi]
         label  'S+ W- E+' style branch label (shoulder, wrist, elbow)
         wrist_singular  True when sin q5 = 0 and q6 was set to 0
    Branches whose acos argument leaves [-1, 1] are dropped (that family cannot reach the pose)."""
    sols = []
    for ls, q1 in group1_shoulder(T06):
        for lw, q5, q6, wsing in group2_wrist(T06, q1):
            for le, q2, q3, q4 in group3_planar(T06, q1, q5, q6):
                sols.append(dict(q=np.array([q1, q2, q3, q4, q5, q6]), label=f"{ls} {lw} {le}",
                                 wrist_singular=wsing))
    return sols


def ik_world(T_world_flange, x):
    """IK for a world flange pose with the base at x."""
    return ik(T06_from_world(T_world_flange, x))


def branch_of(q):
    """Branch label read off a joint vector (the sign triple of the three determinant factors):
    shoulder: sign of the acos term in q1 reconstruction, here taken as the sign of rho; elbow: sign q3;
    wrist: sign q5."""
    q = np.asarray(q, float)
    rho = a2 * np.cos(q[1]) + a3 * np.cos(q[1] + q[2]) + d5 * np.sin(q[1] + q[2] + q[3])
    return f"{'S+' if rho < 0 else 'S-'} {'W+' if q[4] >= 0 else 'W-'} {'E+' if q[2] >= 0 else 'E-'}"


# ---------------------------------------------------------------- quick self-test
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    worst_match, worst_fk, counts = 0.0, 0.0, {}
    for _ in range(2000):
        q = rng.uniform(-pi, pi, 6)
        T = fk_rows(q)
        sols = ik(T)
        counts[len(sols)] = counts.get(len(sols), 0) + 1
        worst_match = max(worst_match, min(np.abs(wrap(s["q"] - q)).max() for s in sols))
        worst_fk = max(worst_fk, max(np.abs(fk_rows(s["q"]) - T).max() for s in sols))
    print(f"2000 random q:  max |q_true - nearest branch| = {worst_match:.1e} rad")
    print(f"                max |FK(branch) - T06|        = {worst_fk:.1e}")
    print(f"                number of branches found: {dict(sorted(counts.items()))}")
