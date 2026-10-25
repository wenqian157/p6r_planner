"""
Analytical fixed-x IK of the UR12e (UR-specific closed form, offset wrist), vectorised.

For base position x the world target T_W is converted to the UR base frame:
    T06 = T_world_urbase(x)^-1  T_W
and the 6R problem is solved in closed form.  Up to 8 branches, indexed by
    b = 4*i_s + 2*i_w + i_e,   sign = +1 for i = 0, -1 for i = 1
      shoulder  i_s : sign of the acos term in q1
      wrist     i_w : sign of q5
      elbow     i_e : sign of q3
Status codes per (pose, branch):
    0 OK, 1 NO_REAL (no real root), 2 JOINT_LIMIT, 3 FK_FAIL (verification failed)
Regularity (sigma_min of the reduced Jacobian) is evaluated separately (maps step).
"""
import numpy as np
from .kinematics import D1, A2, A3, D4, D5, D6, Q_LIM, fk_ur, T_world_urbase, wrap

OK, NO_REAL, JOINT_LIMIT, FK_FAIL = 0, 1, 2, 3
BRANCH_SIGNS = np.array([[s, w, e] for s in (1, -1) for w in (1, -1) for e in (1, -1)])
BRANCH_LABELS = ['S%sW%sE%s' % tuple('+' if v > 0 else '-' for v in sg) for sg in BRANCH_SIGNS]
S5_EPS = 1e-9


def ik_ur(T06, lim=Q_LIM[1:], fk_tol=1e-8):
    """T06: (N,4,4) targets in the UR base frame.
    Returns q (N,8,6) [nan where no real root], status (N,8) int."""
    T06 = np.asarray(T06, float)
    if T06.ndim == 2:
        T06 = T06[None]
    N = len(T06)
    R, p = T06[:, :3, :3], T06[:, :3, 3]
    q = np.full((N, 8, 6), np.nan)
    real = np.ones((N, 8), bool)

    # ---- q1 (shoulder): p5 . z1 = d4
    p5 = p - D6 * R[:, :, 2]
    r = np.hypot(p5[:, 0], p5[:, 1])
    c = D4 / np.maximum(r, 1e-300)
    ok1 = np.abs(c) <= 1.0
    phi = np.arccos(np.clip(c, -1, 1))
    psi = np.arctan2(p5[:, 1], p5[:, 0])

    for b, (ss, sw, se) in enumerate(BRANCH_SIGNS):
        q1 = psi + ss * phi + np.pi / 2
        s1, c1 = np.sin(q1), np.cos(q1)
        z1 = np.stack([s1, -c1, np.zeros(N)], 1)              # axis of joints 2,3,4

        # ---- q5, q6 (wrist):  R^T z1 = (c6 s5, -s6 s5, c5)
        w = np.einsum('nji,nj->ni', R, z1)
        ok5 = np.abs(w[:, 2]) <= 1.0 + 1e-12
        q5 = sw * np.arccos(np.clip(w[:, 2], -1, 1))
        s5 = np.sin(q5)
        sing5 = np.abs(s5) < S5_EPS                           # q6 free: pick 0
        q6 = np.where(sing5, 0.0, np.arctan2(-w[:, 1] / np.where(sing5, 1, s5),
                                             w[:, 0] / np.where(sing5, 1, s5)))

        # ---- planar 3R:  T14 = A1^-1 T06 A6^-1 A5^-1
        A1i = _dh_inv(q1, D1, 0, np.pi / 2)
        A5i = _dh_inv(q5, D5, 0, -np.pi / 2)
        A6i = _dh_inv(q6, D6, 0, 0)
        T14 = A1i @ T06 @ A6i @ A5i
        px, py = T14[:, 0, 3], T14[:, 1, 3]
        c3 = (px ** 2 + py ** 2 - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
        ok3 = np.abs(c3) <= 1.0 + 1e-12
        q3 = se * np.arccos(np.clip(c3, -1, 1))
        q2 = np.arctan2(py, px) - np.arctan2(A3 * np.sin(q3), A2 + A3 * np.cos(q3))
        q4 = np.arctan2(T14[:, 1, 0], T14[:, 0, 0]) - q2 - q3

        qb = wrap(np.stack([q1, q2, q3, q4, q5, q6], 1))
        real[:, b] = ok1 & ok5 & ok3
        q[:, b] = np.where(real[:, b, None], qb, np.nan)

    status = np.where(real, OK, NO_REAL)
    # ---- joint limits (+-pi after wrapping -> only bites with a margin)
    inlim = np.all((q >= lim[:, 0]) & (q <= lim[:, 1]), axis=2)
    status = np.where(real & ~inlim, JOINT_LIMIT, status)
    # ---- independent FK verification of every retained root
    m = status == OK
    if m.any():
        Tk = fk_ur(q[m])
        Tt = np.repeat(T06[:, None], 8, 1)[m]
        e = (np.linalg.norm(Tk[:, :3, 3] - Tt[:, :3, 3], axis=1)
             + np.linalg.norm(Tk[:, :3, :3] - Tt[:, :3, :3], axis=(1, 2)))
        st = status[m]; st[e > fk_tol] = FK_FAIL; status[m] = st
    return q, status


def ik_world(T_W, x, **kw):
    """Whole-body IK slice: world target(s) T_W (N,4,4) with base at x (scalar or (N,))."""
    T_W = np.asarray(T_W, float)
    T_W = T_W[None] if T_W.ndim == 2 else T_W
    x = np.broadcast_to(np.asarray(x, float), (len(T_W),))
    T06 = np.linalg.inv(T_world_urbase(x)) @ T_W
    return ik_ur(T06, **kw)


def _dh_inv(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    N = len(theta)
    Ai = np.zeros((N, 4, 4))
    Ai[:, 0, 0], Ai[:, 0, 1], Ai[:, 0, 3] = ct, st, -a
    Ai[:, 1, 0], Ai[:, 1, 1], Ai[:, 1, 2], Ai[:, 1, 3] = -st * ca, ct * ca, sa, -d * sa
    Ai[:, 2, 0], Ai[:, 2, 1], Ai[:, 2, 2], Ai[:, 2, 3] = st * sa, -ct * sa, ca, -d * ca
    Ai[:, 3, 3] = 1.0
    return Ai


def periodic_representatives(qi, lo, hi):
    """All q + 2k*pi inside [lo, hi] (general case; with the TA's +-pi limits this
    returns a single value except exactly on the seam)."""
    k = np.arange(np.ceil((lo - qi) / (2 * np.pi)), np.floor((hi - qi) / (2 * np.pi)) + 1)
    return qi + 2 * np.pi * k


def branch_of(q):
    """Recover the branch label of an arbitrary joint vector (for round-trip tests)."""
    q = np.atleast_2d(q)
    T = fk_ur(q)
    p5 = T[:, :3, 3] - D6 * T[:, :3, 2]
    psi = np.arctan2(p5[:, 1], p5[:, 0])
    ss = np.sign(wrap(q[:, 0] - np.pi / 2 - psi))
    return np.stack([ss, np.sign(q[:, 4]), np.sign(q[:, 2])], 1)
