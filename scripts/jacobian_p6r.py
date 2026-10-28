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


# ---------------------------------------------------------------- course subchain Jacobian 3J5 (arm only)
from fk_dh_p6r import DH as DH_P6R, T_BASE as T_BASE_DH
from fk_dh import dh_row


def dh_frame(q, k):
    """Pose of UR DH frame k (k = 0..6) in the world, from the validated P6R DH table.
    P6R row 0 is the slide, so UR frame k is reached after rows 0..k."""
    T = T_BASE_DH.copy()
    for i in range(k + 1):
        jtype, off, d, a, alpha = DH_P6R[i]
        T = T @ (dh_row(off, d + q[i], a, alpha) if jtype == "P" else dh_row(q[i] + off, d, a, alpha))
    return T


def jacobian_3J5(q, point="O5"):
    """3J5 of the arm (x fixed), lecture-5 style: velocity point on the wrist, both blocks expressed in
    UR DH frame 3.  6 x 6, columns q1..q6, [v; w].

    The UR12e wrist is offset: axes 4, 5 meet at O4 (DH frame 4 origin), axes 5, 6 meet at O5 (DH frame 5
    origin), 0.11985 m apart. No point lies on all three, so `point` chooses which pair is decoupled:
        point="O5": columns 5, 6 lose their linear part, column 4 keeps one (|a4| = d5)
        point="O4": columns 4, 5 lose their linear part, column 6 keeps one (|a6| = d5)

    Block form  [A C; B W]:
        A (3x3) how q1..q3 move the point    W (3x3) how q4..q6 rotate the tool (same for both points)
        B (3x3) how q1..q3 rotate the tool   C (3x3) the offset-wrist leak, one column of length d5
    For an intersecting wrist C = 0 and det 3J5 = det A * det W.  Here det 3J5 = det A * det(W - B A^-1 C),
    and numerically: det W = 0 <-> wrist family (q5 = 0), det A(O4) = 0 <-> elbow family (q3 = 0, pi),
    det A(O5) = 0 at the shoulder family, but det A = 0 alone is neither necessary nor sufficient.
    det 3J5 = det 0J_red for every q (point shift and frame rotation have unit determinant)."""
    k = {"O5": 5, "O4": 4}[point]
    O = dh_frame(q, k)[:3, 3]
    R3 = dh_frame(q, 3)[:3, :3]
    J0 = jacobian(q, point=O)[:, 1:]           # 0J of the arm: world frame, velocity point O
    return np.kron(np.eye(2), R3.T) @ J0       # blockdiag(R3^T, R3^T) @ 0J


def blocks_3J5(J):
    return dict(A=J[:3, :3], C=J[:3, 3:], B=J[3:, :3], W=J[3:, 3:])


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

    q_deg = [10, -60, 90, -120, -90, 30]
    q = np.r_[0.5, np.deg2rad(q_deg)]
    J = jacobian(q)
    fd = np.abs(J - jacobian_fd(q)).max()
    slide_ok = np.allclose(J[:, 0], [1, 0, 0, 0, 0, 0])
    print("=== 6 x 7 geometric Jacobian at x = 0.5 m, q = (10,-60,90,-120,-90,30) deg,  rows [vx vy vz wx wy wz] ===")
    print_J(J, NAMES)
    print(f"\nfinite-difference check: max |J - J_fd| = {fd:.2e}")
    print("slide column is [1 0 0 0 0 0]: pure x translation, independent of q  ->", slide_ok)

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
    results = []
    for name, c in cases:
        qq = np.r_[c[0], np.deg2rad(c[1:])]
        r = analyse(qq)
        results.append((name, c, r))
        print(f"{name:40s} {r['det_red']:11.2e} {r['smin_red']:9.2e} {r['rank_red']:8d} {r['smin_full']:10.2e} {r['rank_full']:9d}  "
              f"{np.array2string(r['lost'], precision=2, suppress_small=True):28s} {r['lost_x']:.2f}")
    print("\nreading: rank_full = 6 while rank_red = 5 means the slide rescues the lost direction (|u_x| > 0);")
    print("         rank_full = 5 means the singularity belongs to the whole body (lost direction has no x component).")

    # ------------------------------------------------------------ write results/jacobian_p6r.md
    from datetime import date
    rows = ["vx", "vy", "vz", "wx", "wy", "wz"]
    md = ["# Geometric Jacobian of the P6R", "",
          f"Date: {date.today().isoformat()}  ", "Script: `scripts/jacobian_p6r.py`  ",
          "Convention: lecture 5, twist = [v; w] in the world frame, velocity point = flange origin.  ",
          "Column k: revolute [z_k x (p_t - p_k); z_k], prismatic [z_k; 0].", "",
          f"## Full 6 x 7 Jacobian at x = 0.5 m, q = {q_deg} deg", "",
          "| | " + " | ".join(NAMES) + " |", "|---|" + "---|" * 7]
    md += ["| " + lbl + " | " + " | ".join(f"{v: .4f}" for v in row) + " |" for lbl, row in zip(rows, J)]
    md += ["", f"Finite-difference check: max |J - J_fd| = {fd:.2e}.  ",
           f"Slide column equals [1 0 0 0 0 0]: {slide_ok} (pure x translation, independent of q).", "",
           "## Reduced 6 x 6 (x fixed) vs full 6 x 7 at representative configurations", "",
           "| configuration | x [m], q [deg] | det J_red | sigma_min(J_red) | rank J_red | sigma_min(J) | rank J | lost direction u [v; w] | abs(u_x) |",
           "|---|---|---|---|---|---|---|---|---|"]
    for name, c, r in results:
        md.append(f"| {name} | {c} | {r['det_red']:.2e} | {r['smin_red']:.2e} | {r['rank_red']} | {r['smin_full']:.2e} | "
                  f"{r['rank_full']} | {np.array2string(r['lost'], precision=2, suppress_small=True)} | {r['lost_x']:.2f} |")
    md += ["", "Reading: rank J = 6 with rank J_red = 5 means the slide rescues the lost direction (abs(u_x) > 0). "
           "rank J = 5 means the singularity belongs to the whole body: the lost direction has no x component.", ""]
    out = Path(__file__).resolve().parent.parent / "results" / "jacobian_p6r.md"
    out.parent.mkdir(exist_ok=True)
    out.write_text("\n".join(md), encoding="utf-8")
    print(f"\nwritten to {out.relative_to(out.parent.parent)}")
