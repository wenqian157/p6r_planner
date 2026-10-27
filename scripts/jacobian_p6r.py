"""Geometric Jacobian of the P6R (UMR x-slide + UR12e), lecture-5 convention.

Twist ordering: linear first,  [v; w]  (world frame).
Column k, evaluated at the current configuration q:
    revolute  joint:  [ z_k x (p_t - p_k) ;  z_k ]     z_k = axis direction, p_k = point on the axis
    prismatic joint:  [ z_k               ;  0   ]
Full Jacobian:     J      6 x 7, columns (x, q1, ..., q6)
Reduced Jacobian:  J_red  6 x 6, columns (q1, ..., q6)  = J with the slide column removed (x fixed)

Singularity measures: det(J_red), sigma_min, rank.  A singularity of J_red is a singularity of the
whole body only if the lost task direction u (left null vector of J_red) has no component along
the slide column [x_hat; 0], i.e. u . [1,0,0,0,0,0] = 0.

Run from the repo root:
    python scripts/jacobian_p6r.py                           -> demo + singularity table
    python scripts/jacobian_p6r.py 0.5 10 -60 90 -120 -90 30  -> print J (6x7) and J_red (6x6) at x [m], q [deg]
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_urdf_p6r import CHAIN, fk as fk_p6r
from fk_urdf import origin, rot_axis
from fk_urdf_p6r import trans_axis

np.set_printoptions(precision=4, suppress=True, linewidth=140)
pi = np.pi
NAMES = ["x", "q1", "q2", "q3", "q4", "q5", "q6"]


# ---------------------------------------------------------------- joint axes at q
def joint_axes(q):
    """Walk the chain once. Returns ([(type, z_k, p_k)], p_tool, R_tool) in the world frame at q.
    z_k = axis direction, p_k = origin of joint frame k (a point on the axis)."""
    T, k, axes = np.eye(4), 0, []
    for name, xyz, rpy, jtype, axis in CHAIN:
        T = T @ origin(xyz, rpy)
        if jtype is not None:
            z = T[:3, :3] @ (np.asarray(axis, float) / np.linalg.norm(axis))
            axes.append((jtype, z, T[:3, 3].copy()))
            T = T @ (rot_axis(axis, q[k]) if jtype == "R" else trans_axis(axis, q[k]))
            k += 1
    return axes, T[:3, 3].copy(), T[:3, :3].copy()


# ---------------------------------------------------------------- Jacobians
def jacobian(q, point=None):
    """6 x 7 geometric Jacobian [v; w] of the point `point` (default: flange origin), world frame."""
    axes, p_tool, _ = joint_axes(q)
    p = p_tool if point is None else np.asarray(point, float)
    J = np.zeros((6, 7))
    for k, (jtype, z, pk) in enumerate(axes):
        if jtype == "R":
            J[:3, k] = np.cross(z, p - pk)
            J[3:, k] = z
        else:
            J[:3, k] = z
    return J


def jacobian_reduced(q, point=None):
    """6 x 6 Jacobian of the arm for fixed x: drop the slide column."""
    return jacobian(q, point)[:, 1:]


# ---------------------------------------------------------------- numerical check
def jacobian_fd(q, h=1e-6):
    """Finite-difference Jacobian from the FK: dp/dq and w from dR R^T."""
    J = np.zeros((6, 7))
    for k in range(7):
        dq = np.zeros(7); dq[k] = h
        Tp, Tm = fk_p6r(q + dq), fk_p6r(q - dq)
        J[:3, k] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
        dR = (Tp[:3, :3] - Tm[:3, :3]) / (2 * h) @ fk_p6r(q)[:3, :3].T    # = [w]_x
        J[3:, k] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


# ---------------------------------------------------------------- singularity analysis
def analyse(q):
    J = jacobian(q)
    Jr = J[:, 1:]
    s_full = np.linalg.svd(J, compute_uv=False)
    U, s_red, _ = np.linalg.svd(Jr)
    u = U[:, -1]                                   # task direction lost first by the arm
    return dict(det_red=np.linalg.det(Jr), smin_red=s_red[-1], rank_red=np.linalg.matrix_rank(Jr, tol=1e-9),
                smin_full=s_full[-1], rank_full=np.linalg.matrix_rank(J, tol=1e-9),
                lost=u, lost_x=abs(u[0]))


def print_J(J, cols):
    print("        " + "".join(f"{n:>10s}" for n in cols))
    for lbl, row in zip(["vx", "vy", "vz", "wx", "wy", "wz"], J):
        print(f"{lbl:6s}  " + "".join(f"{v:10.4f}" for v in row))


if __name__ == "__main__":
    if len(sys.argv) == 8:
        q = np.r_[float(sys.argv[1]), np.deg2rad([float(v) for v in sys.argv[2:]])]
        print(f"x = {q[0]} m,  q = {np.rad2deg(q[1:])} deg\n")
        print("full 6 x 7 Jacobian, columns (x, q1..q6):")
        print_J(jacobian(q), NAMES)
        Jr = jacobian_reduced(q)
        print("\nreduced 6 x 6 Jacobian (x fixed), columns (q1..q6):")
        print_J(Jr, NAMES[1:])
        print(f"\ndet J_red = {np.linalg.det(Jr):.4e}   sigma_min(J_red) = {np.linalg.svd(Jr, compute_uv=False)[-1]:.4e}"
              f"   sigma_min(J) = {np.linalg.svd(jacobian(q), compute_uv=False)[-1]:.4e}")
        sys.exit()

    q = np.r_[0.5, np.deg2rad([10, -60, 90, -120, -90, 30])]
    J = jacobian(q)
    print("=== 6 x 7 geometric Jacobian at x = 0.5 m, q = (10,-60,90,-120,-90,30) deg,  rows [vx vy vz wx wy wz] ===")
    print("        " + "".join(f"{n:>10s}" for n in NAMES))
    for lbl, row in zip(["vx", "vy", "vz", "wx", "wy", "wz"], J):
        print(f"{lbl:6s}  " + "".join(f"{v:10.4f}" for v in row))
    print(f"\nfinite-difference check: max |J - J_fd| = {np.abs(J - jacobian_fd(q)).max():.2e}")
    print("slide column is [1 0 0 0 0 0]: pure x translation, independent of q  ->",
          np.allclose(J[:, 0], [1, 0, 0, 0, 0, 0]))

    print("\n=== reduced 6 x 6 (x fixed) vs full 6 x 7 at representative configurations ===")
    cases = [
        ("regular",                                   [0.5, 10, -60, 90, -120, -90, 30]),
        ("wrist sing.  q5 = 0",                       [0.5, 10, -60, 90, -120,   0, 30]),
        ("elbow sing.  q3 = 0, arm plane = x-z",      [0.5,  0, -45,  0,  -45, -90,  0]),
        ("elbow sing.  q3 = 0, arm plane = y-z",      [0.5, 90, -45,  0,  -45, -90,  0]),
        ("elbow sing.  q3 = 0, arm vertical",         [0.5,  0, -90,  0,    0, -90,  0]),
        ("q = 0  (elbow + wrist singular)",           [0.5,  0,   0,  0,    0,   0,  0]),
    ]
    print(f"{'configuration':40s} {'det J_red':>11s} {'smin_red':>9s} {'rank_red':>8s} {'smin_full':>10s} {'rank_full':>9s}  lost dir [v;w] (arm)        |u_x|")
    for name, c in cases:
        qq = np.r_[c[0], np.deg2rad(c[1:])]
        r = analyse(qq)
        print(f"{name:40s} {r['det_red']:11.2e} {r['smin_red']:9.2e} {r['rank_red']:8d} {r['smin_full']:10.2e} {r['rank_full']:9d}  "
              f"{np.array2string(r['lost'], precision=2, suppress_small=True):28s} {r['lost_x']:.2f}")
    print("\nreading: rank_full = 6 while rank_red = 5 means the slide rescues the lost direction (|u_x| > 0);")
    print("         rank_full = 5 means the singularity belongs to the whole body (lost direction has no x component).")
