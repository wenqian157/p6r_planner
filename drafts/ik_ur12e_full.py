"""DRAFT (Report section 3) - complete analytical IK of the UR12e for a fixed base position x.

Continues scripts/ik_ur12e.py (frames + step 1 already there) with steps 2-5, then assembles the
full solver: all 8 branches, wrapping to (-pi, pi], +-pi joint limits, FK verification, branch labels.
Vectorised over N target poses from the start (the maps and path steps call it millions of times).

UR recipe (offset wrist: axes 4,5,6 do NOT intersect -> no Pieper split):
    step 1  theta1      from the position of O5 = p06 - d6 z6           +-  shoulder  (S)
    step 2  theta5      from the orientation, given theta1              +-  wrist     (W)
    step 3  theta6      from the orientation, given theta1, theta5      unique (free if s5 = 0)
    step 4  theta2,3    planar 2R in the plane of axes 2,3,4            +-  elbow     (E)
    step 5  theta4      remaining rotation of the planar chain          unique
    -> 2 x 2 x 2 = 8 branches, index b = 4*iS + 2*iW + iE

Run from the repo root:
    python drafts/ik_ur12e_full.py
"""
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ik_ur12e import d1, a2, a3, d4, d5, d6, T06_from_world, fk_rows
try:                                                     # older scripts/ik_ur12e.py (step 1 only)
    from ik_ur12e import step1_theta1
except ImportError:                                      # newer version: group1_shoulder returns [(label, q1)]
    from ik_ur12e import group1_shoulder
    step1_theta1 = lambda T: [q for _, q in group1_shoulder(T)]
from fk_dh import dh_row, origin
from fk_urdf_p6r import fk as fk_p6r                    # independent reference model (URDF chain)
from jacobian_p6r import jacobian, jacobian_reduced
from umr_params import MOUNT_XYZ

pi = np.pi
np.set_printoptions(precision=4, suppress=True, linewidth=140)

OK, NO_REAL, JOINT_LIMIT, FK_FAIL = 0, 1, 2, 3
STATUS = {OK: "ok", NO_REAL: "no real IK", JOINT_LIMIT: "joint limit", FK_FAIL: "FK check failed"}
SIGNS = np.array([[s, w, e] for s in (1, -1) for w in (1, -1) for e in (1, -1)])   # (8, 3)
LABELS = ["S%s W%s E%s" % tuple("+" if v > 0 else "-" for v in sg) for sg in SIGNS]
Q_LIM = np.array([[-pi, pi]] * 6)          # project limits (proposal): +-pi on every arm joint
S5_EPS = 1e-9


def wrap(a):
    """Wrap to (-pi, pi]."""
    return pi - np.mod(pi - a, 2 * pi)


# ================================================================ steps (vectorised)
def theta1(T06, sS):
    """Step 1.  O5 lies in the arm plane, which is offset by d4 from axis 1.
    Seen from above:  r = |p05_xy|,  sin(angle between O5 and the plane normal) ... gives
        theta1 = atan2(p05y, p05x) + sS * acos(d4 / r) + pi/2        real iff r >= d4"""
    p05 = T06[:, :3, 3] - d6 * T06[:, :3, 2]
    r = np.hypot(p05[:, 0], p05[:, 1])
    c = d4 / np.maximum(r, 1e-300)
    return np.arctan2(p05[:, 1], p05[:, 0]) + sS * np.arccos(np.clip(c, -1, 1)) + pi / 2, np.abs(c) <= 1


def theta5_6(T06, t1, sW):
    """Steps 2 + 3.  Axes 2, 3, 4 are all parallel to z1 = (s1, -c1, 0) in frame 0.
    Expressed in frame 6 that axis is  R06^T z1 = R56^T R45^T e_y = (s5 c6, -s5 s6, c5):
        theta5 = sW * acos(w_z)                      (wrist flip = sign of theta5)
        theta6 = atan2(-w_y / s5, w_x / s5)          (free when s5 = 0 -> wrist singularity, pick 0)"""
    z1 = np.stack([np.sin(t1), -np.cos(t1), np.zeros_like(t1)], 1)
    w = np.einsum("nji,nj->ni", T06[:, :3, :3], z1)
    t5 = sW * np.arccos(np.clip(w[:, 2], -1, 1))
    s5 = np.sin(t5)
    sing = np.abs(s5) < S5_EPS
    s5s = np.where(sing, 1.0, s5)
    t6 = np.where(sing, 0.0, np.arctan2(-w[:, 1] / s5s, w[:, 0] / s5s))
    return t5, t6


def theta2_3_4(T06, t1, t5, t6, sE):
    """Steps 4 + 5.  Peel off joints 1, 5, 6:  T14 = A1^-1 T06 A6^-1 A5^-1 = A2 A3 A4.
    Its origin (px, py, d4) is O4 in frame 1: a planar 2R with links a2, a3 (law of cosines),
        c3 = (px^2 + py^2 - a2^2 - a3^2) / (2 a2 a3),   theta3 = sE * acos(c3)
        theta2 = atan2(py, px) - atan2(a3 s3, a2 + a3 c3)
    and the in-plane angle of x4 gives  theta2 + theta3 + theta4 = atan2(R14[1,0], R14[0,0])."""
    A1i = np.linalg.inv(_dh(t1, d1, 0.0, pi / 2))
    A5i = np.linalg.inv(_dh(t5, d5, 0.0, -pi / 2))
    A6i = np.linalg.inv(_dh(t6, d6, 0.0, 0.0))
    T14 = A1i @ T06 @ A6i @ A5i
    px, py = T14[:, 0, 3], T14[:, 1, 3]
    c3 = (px ** 2 + py ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
    t3 = sE * np.arccos(np.clip(c3, -1, 1))
    t2 = np.arctan2(py, px) - np.arctan2(a3 * np.sin(t3), a2 + a3 * np.cos(t3))
    t4 = np.arctan2(T14[:, 1, 0], T14[:, 0, 0]) - t2 - t3
    return t2, t3, t4, np.abs(c3) <= 1 + 1e-12


def _dh(theta, d, a, alpha):
    """Standard DH row Rz(theta) Tz(d) Tx(a) Rx(alpha) for an array of theta -> (N,4,4)
    (same formula as fk_dh.dh_row, written out so it runs on arrays)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    A = np.zeros(np.shape(theta) + (4, 4))
    A[..., 0, :] = np.stack([ct, -st * ca, st * sa, a * ct], -1)
    A[..., 1, :] = np.stack([st, ct * ca, -ct * sa, a * st], -1)
    A[..., 2, 1], A[..., 2, 2], A[..., 2, 3], A[..., 3, 3] = sa, ca, d, 1.0
    return A


DH_ROWS = [(d1, 0.0, pi / 2), (0.0, a2, 0.0), (0.0, a3, 0.0), (d4, 0.0, pi / 2), (d5, 0.0, -pi / 2), (d6, 0.0, 0.0)]


def fk_rows_v(q):
    """Vectorised fk_rows: q (N,6) -> T06 (N,4,4)."""
    T = np.broadcast_to(np.eye(4), (len(q), 4, 4))
    for i, (d, a, al) in enumerate(DH_ROWS):
        T = T @ _dh(q[:, i], d, a, al)
    return T


# ================================================================ full solver
def ik_6r(T06, lim=Q_LIM, fk_tol=1e-8):
    """All IK branches of the 6R arm.  T06 (N,4,4) or (4,4) -> q (N,8,6) [nan if none], status (N,8)."""
    T06 = np.asarray(T06, float)
    T06 = T06[None] if T06.ndim == 2 else T06
    N = len(T06)
    q = np.full((N, 8, 6), np.nan)
    real = np.zeros((N, 8), bool)
    for b, (sS, sW, sE) in enumerate(SIGNS):
        t1, ok1 = theta1(T06, sS)
        t5, t6 = theta5_6(T06, t1, sW)
        t2, t3, t4, ok3 = theta2_3_4(T06, t1, t5, t6, sE)
        real[:, b] = ok1 & ok3
        q[:, b] = np.where(real[:, b, None], wrap(np.stack([t1, t2, t3, t4, t5, t6], 1)), np.nan)

    status = np.where(real, OK, NO_REAL)
    inlim = np.all((q >= lim[:, 0]) & (q <= lim[:, 1]), axis=2)     # with +-pi always true after wrap
    status[real & ~inlim] = JOINT_LIMIT
    # independent check: every kept root must reproduce the target through the DH rows
    n, b = np.nonzero(status == OK)
    if len(n):
        Tk = fk_rows_v(q[n, b])
        err = np.linalg.norm(Tk[:, :3, 3] - T06[n, :3, 3], axis=1) + \
              np.linalg.norm(Tk[:, :3, :3] - T06[n, :3, :3], axis=(1, 2))
        status[n[err > fk_tol], b[err > fk_tol]] = FK_FAIL
    return q, status


def ik_world(T_world_flange, x, **kw):
    """Whole-body IK slice at base position x: world flange pose -> UR branches."""
    return ik_6r(T06_from_world(T_world_flange, x), **kw)


def branch_of(q):
    """Label (sS, sW, sE) of an arbitrary joint vector, from the same geometry the solver uses."""
    T = fk_rows(q)
    p05 = T[:3, 3] - d6 * T[:3, 2]
    sS = np.sign(wrap(q[0] - pi / 2 - np.arctan2(p05[1], p05[0])))
    return np.array([sS, np.sign(q[4]), np.sign(q[2])])


# ================================================================ checks
if __name__ == "__main__":
    rng = np.random.default_rng(2)

    # ---- (0) her step 1 and the vectorised one agree
    Q = rng.uniform(-pi, pi, (500, 6))
    T = np.stack([fk_rows(qq) for qq in Q])
    t1v = np.stack([theta1(T, s)[0] for s in (1, -1)], 1)
    t1s = np.array([step1_theta1(Ti) for Ti in T])
    print(f"(0) vectorised DH rows vs scripts/ik_ur12e.fk_rows: max diff = {np.abs(fk_rows_v(Q) - T).max():.1e}")
    print(f"(0) step 1, vectorised vs scripts/ik_ur12e.py: max diff = {np.abs(wrap(t1v - t1s)).max():.1e} rad")

    # ---- (a) round trip on random joint vectors
    N = 200_000
    Q = rng.uniform(-pi, pi, (N, 6))
    T = fk_rows_v(Q)
    t0 = time.perf_counter()
    qs, st = ik_6r(T)
    dt = time.perf_counter() - t0
    dq = np.abs(wrap(qs - Q[:, None])).max(2)
    dq[st != OK] = np.inf
    best, b_hit = dq.min(1), dq.argmin(1)
    reg = np.abs(np.sin(Q[:, 4])) > 1e-3                         # away from the wrist singularity
    lab = np.array([branch_of(qq) for qq in Q[:20_000]])
    print(f"\n(a) round trip, {N} random q in [-pi, pi]^6   ({dt:.2f} s, {N / dt:,.0f} poses/s, 8 branches each)")
    print(f"    original q recovered (|dq| < 1e-6)       : {np.mean(best[reg] < 1e-6) * 100:.3f} %")
    print(f"    worst recovery error                     : {best[reg].max():.1e} rad")
    print(f"    every returned root reproduces the pose  : {(st != FK_FAIL).all()}")
    print(f"    recovered root has q's own label         : {np.mean((SIGNS[b_hit[:20_000]] == lab).all(1)[reg[:20_000]]) * 100:.3f} %")
    print(f"    number of OK branches per pose (0..8)    : {np.bincount((st == OK).sum(1), minlength=9)}")

    # ---- (b) label persistence along smooth paths
    viol = tot = 0
    near = []                                     # at each switch: |sin q5|, |sin q3|, r/d4 - 1
    for _ in range(100):
        qa, qb = rng.uniform(-2.5, 2.5, 6), rng.uniform(-2.5, 2.5, 6)
        P = qa + np.linspace(0, 1, 300)[:, None] * (qb - qa)
        Tp = fk_rows_v(P)
        Qp, Sp = ik_6r(Tp)
        p05 = Tp[:, :3, 3] - d6 * Tp[:, :3, 2]
        rr = np.hypot(p05[:, 0], p05[:, 1]) / d4 - 1      # 0 at the shoulder singularity (S+ meets S-)
        for b in range(8):
            for k in range(1, len(P)):
                if Sp[k, b] != OK or Sp[k - 1, b] != OK:
                    continue
                d = np.linalg.norm(wrap(Qp[k] - Qp[k - 1, b]), axis=1)
                d[Sp[k] != OK] = np.inf
                tot += 1
                if np.argmin(d) != b:
                    viol += 1
                    near.append([abs(np.sin(Qp[k, b, 4])), abs(np.sin(Qp[k, b, 2])), rr[k]])
    print(f"\n(b) label persistence: nearest root at the next step has the same label in "
          f"{100 * (1 - viol / tot):.3f} % of {tot} steps")
    near = np.array(near)
    if len(near):
        print(f"    the {viol} label switches happen next to a singularity where two branches meet:")
        sh, wr, el = near[:, 2] < 0.05, near[:, 0] < 0.2, near[:, 1] < 0.2
        print(f"      shoulder  r < 1.05 d4   (S+ meets S-) : {np.mean(sh) * 100:5.1f} %")
        print(f"      wrist     |sin q5| < 0.2 (W+ meets W-) : {np.mean(wr & ~sh) * 100:5.1f} %  (not already shoulder)")
        print(f"      elbow     |sin q3| < 0.2 (E+ meets E-) : {np.mean(el & ~sh & ~wr) * 100:5.1f} %  (not already above)")
        print(f"      none of the three                      : {np.mean(~(sh | wr | el)) * 100:5.1f} %")

    # ---- (c) one representative tool pose, all branches, world frame, x = 0.5 m (same x as sections 1-2)
    x = 0.5
    T_star = fk_p6r(np.r_[x, np.deg2rad([10, -60, 90, -120, -90, 30])])
    qW, sW = ik_world(T_star, x)
    print(f"\n(c) target = flange pose of section 1.4 (x = {x} m, q = [10 -60 90 -120 -90 30] deg)")
    print(f"    {'branch':10s} {'q1..q6 [deg]':52s} {'|dp| URDF [m]':>14s} {'sig_min(J_red)':>15s} {'sig_min(J)':>11s}")
    for b in range(8):
        if sW[0, b] != OK:
            print(f"    {LABELS[b]:10s} {STATUS[sW[0, b]]}")
            continue
        Qb = np.r_[x, qW[0, b]]
        e = np.linalg.norm(fk_p6r(Qb)[:3, 3] - T_star[:3, 3])
        sr = np.linalg.svd(jacobian_reduced(Qb), compute_uv=False)[-1]
        sf = np.linalg.svd(jacobian(Qb), compute_uv=False)[-1]
        print(f"    {LABELS[b]:10s} {str(np.round(np.rad2deg(qW[0, b]), 1)):52s} {e:14.1e} {sr:15.4f} {sf:11.4f}")

    # ---- (d) bridge to sections 4-5: same world pose, the base slides -> which branches exist?
    xs = np.round(np.arange(-1.5, 1.51, 0.25), 2)
    Sx = np.array([ik_world(T_star, xx)[1][0] for xx in xs])
    print(f"\n(d) same world target, base position swept (o = ok, . = no real IK)")
    print("    x [m]  " + " ".join(f"{L:>9s}" for L in LABELS))
    for xx, srow in zip(xs, Sx):
        print(f"    {xx:+5.2f}  " + " ".join(f"{'o' if s == OK else '.':>9s}" for s in srow))
