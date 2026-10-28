"""Step 4D: can the slide rescue an arm singularity?  Several configuration sets, three tests each.

Each family is taken at two base yaws, because the lost direction u turns with the arm:
  elbow    (q3 = 0)   u = radial in the arm plane          -> u_x large at yaw 0,   u_x = 0 for the vertical arm
  wrist    (q5 = 0)   u = sideways, normal to the arm plane -> u_x = 0 at yaw 0,     u_x large at yaw 90
  shoulder (O5 above the shoulder line, |O5 - axis1|_xy = d4)  same as the wrist

Test 0  x sweep with the arm frozen:  sigma_min(J_red) does not move at all.  J_red has no x in it, so
        sliding the base by itself never takes the arm off a singularity.  (negative check)
Test 1  velocity level:  ask the arm alone (J_red), then the whole body (J), for a joint velocity that
        produces the lost direction u.  Residual 0 = can do it.  This is the rank test of the 6 x 7 J.
Test 2  position level:  hold the tool pose T*, slide the base to x, re-solve the arm by numerical IK,
        and watch sigma_min(J_red(q(x))).  Rescue = pose still held AND sigma_min > 0.
        The elbow family is a reach boundary, so test 2 agrees with test 1.  Wrist and shoulder are
        interior singularities: the pose stays reachable, and whether the arm leaves the singular set
        depends on whether that set is invariant under the x self-motion (see the output).

Run from the repo root:   python scripts/step04d_rescue_by_x.py
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jacobian_p6r import jacobian, jacobian_reduced, dh_frame
from umr_params import MOUNT_XYZ
from fk_urdf_p6r import fk

np.set_printoptions(precision=4, suppress=True, linewidth=140)
pi = np.pi
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]
wrap = lambda a: (a + pi) % (2 * pi) - pi


class Tee:
    """Print to the terminal and keep a copy, so the output can be written to results/ as is."""
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()

# ------------------------------------------------ the configuration sets (x in m, arm in deg)
D4 = 0.17415
Z1 = MOUNT_XYZ[2] + 0.1807                                      # height of axis 1


def shoulder_q4(q1, q2=55.0, q3=90.0, q5=-90.0, q6=0.0, lo=110.0, hi=114.0):
    """Bisection on q4 for det J_red = 0 with elbow and wrist away from their own families."""
    f = lambda q4: np.linalg.det(jacobian_reduced(np.r_[0.5, np.deg2rad([q1, q2, q3, q4, q5, q6])]))
    a, b = lo, hi
    for _ in range(60):
        m = 0.5 * (a + b)
        a, b = (a, m) if f(a) * f(m) < 0 else (m, b)
    return 0.5 * (a + b)


def r_O5(q):
    """Horizontal distance of O5 (axes 5, 6) from axis 1, minus d4: zero on the shoulder family."""
    O5 = dh_frame(q, 5)[:3, 3]
    return np.hypot(O5[0] - q[0], O5[1]) - D4


# (name, x0, arm q [deg], watched joint index 0..5 or None, watch label, watch function)
q4s = shoulder_q4(0.0)
SETS = [
    ("elbow     yaw 0   ", 0.5, [0, -45, 0, -45, -90, 0],   2, "q3 [deg]", lambda q: np.rad2deg(q[3])),
    ("elbow     vertical", 0.5, [0, -90, 0,   0, -90, 0],   2, "q3 [deg]", lambda q: np.rad2deg(q[3])),
    ("wrist     yaw 90  ", 0.5, [90, -60, 90, -120, 0, 30], 4, "q5 [deg]", lambda q: np.rad2deg(q[5])),
    ("wrist     yaw 0   ", 0.5, [0, -60, 90, -120, 0, 30],  4, "q5 [deg]", lambda q: np.rad2deg(q[5])),
    ("shoulder  yaw 90  ", 0.5, [90, 55, 90, q4s, -90, 0],  3, "r_O5-d4 [m]", r_O5),
    ("shoulder  yaw 0   ", 0.5, [0, 55, 90, q4s, -90, 0],   3, "r_O5-d4 [m]", r_O5),
]
X_SWEEP = np.linspace(-0.3, 0.3, 13)       # base displacement from x0 in test 2
NUDGE = 0.01                               # rad, added to the watched joint before IK: the singular point is a stationary
                                           # point of the lost direction, so Newton needs a push to leave it
# -----------------------------------------------------------------------------------------------


def pose_err(T, T_star):
    """[dp; dtheta] from T to T_star, world frame, at the flange (same convention as the Jacobian)."""
    dp = T_star[:3, 3] - T[:3, 3]
    dR = T_star[:3, :3] @ T[:3, :3].T
    ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
    w = np.zeros(3) if ang < 1e-12 else ang / (2 * np.sin(ang)) * np.array(
        [dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
    return np.r_[dp, w]


def ik_arm(x, q_arm, T_star, iters=500, lam=1e-2, tol=1e-9):
    """Damped least squares on the arm only (x fixed).  Returns (q_arm, residual norm)."""
    q = q_arm.copy()
    for _ in range(iters):
        e = pose_err(fk(np.r_[x, q]), T_star)
        if np.linalg.norm(e) < tol:
            break
        Jr = jacobian(np.r_[x, q])[:, 1:]
        q = q + Jr.T @ np.linalg.solve(Jr @ Jr.T + lam ** 2 * np.eye(6), e)
    return wrap(q), np.linalg.norm(pose_err(fk(np.r_[x, q]), T_star))


for name, x0, q_deg, kw, wlab, wfn in SETS:
    q0 = np.r_[x0, np.deg2rad(q_deg)]
    J = jacobian(q0)
    J_red = J[:, 1:]
    U, s_red, _ = np.linalg.svd(J_red)
    u = U[:, -1]
    print("=" * 100)
    print(f"{name}:  x = {x0} m,  q = {np.round(q_deg, 2)} deg")
    print(f"  sigma_min(J_red) = {s_red[-1]:.2e}     sigma_min(J) = {smin(J):.4f}")
    print(f"  lost direction u = {u}   [vx vy vz wx wy wz]     |u_x| = {abs(u[0]):.4f}")

    # ---------------------------------------------- test 0: x sweep, arm frozen
    xs = np.linspace(-1.5, 1.5, 7)
    s_frozen = [smin(jacobian(np.r_[x, q0[1:]])[:, 1:]) for x in xs]
    print(f"\n  test 0  arm frozen, x = {xs}")
    print(f"          sigma_min(J_red) = {np.array(s_frozen)}   -> unchanged: J_red does not depend on x")

    # ---------------------------------------------- test 1: velocity level
    qd_arm, *_ = np.linalg.lstsq(J_red, u, rcond=None)
    qd_full, *_ = np.linalg.lstsq(J, u, rcond=None)
    r_arm, r_full = np.linalg.norm(J_red @ qd_arm - u), np.linalg.norm(J @ qd_full - u)
    print(f"\n  test 1  produce the task velocity u (|u| = 1):")
    print(f"          arm alone  (J_red qd = u):  best residual = {r_arm:.4f}")
    print(f"          whole body (J qd = u)    :  best residual = {r_full:.4f},   base speed xdot = {qd_full[0]:.4f} m/s")
    print("          ->", "the slide supplies what the arm cannot" if r_full < 1e-6 else "the slide cannot help either")

    # ---------------------------------------------- test 2: position level, hold the pose, slide the base
    T_star = fk(q0)
    print(f"\n  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by {NUDGE} rad on q{kw + 1})")
    print(f"          {'x - x0 [m]':>11s} {'pose error':>11s} {'held?':>6s} {'sigma_min(J_red)':>17s} {wlab:>12s}   q [deg]")
    rows = {}
    for direction in (-1, +1):                       # continuation outward from x0 on each side
        q_arm = q0[1:].copy(); q_arm[kw] += NUDGE
        for dx in sorted(X_SWEEP[X_SWEEP * direction > 0], key=abs):
            q_arm, res = ik_arm(x0 + dx, q_arm, T_star)
            rows[dx] = (res, q_arm)
    q_n = q0[1:].copy(); q_n[kw] += NUDGE
    q_c, res_c = ik_arm(x0, q_n, T_star)
    rows[0.0] = (res_c, q_c)
    for dx in sorted(rows):
        res, q_arm = rows[dx]
        held = res < 1e-6
        s = smin(jacobian(np.r_[x0 + dx, q_arm])[:, 1:])
        print(f"          {dx:11.3f} {res:11.2e} {'yes' if held else 'no':>6s} {s:17.4f} {wfn(np.r_[x0 + dx, q_arm]):12.4f}   "
              f"{np.array2string(np.rad2deg(q_arm), precision=1, suppress_small=True)}")
    ok = [dx for dx, (res, q_arm) in rows.items() if dx != 0 and res < 1e-6
          and smin(jacobian(np.r_[x0 + dx, q_arm])[:, 1:]) > 1e-3]
    print("          ->", f"rescued for x - x0 in {{{min(ok):+.2f} .. {max(ok):+.2f}}} m: pose held and the arm is regular"
          if ok else ("no x holds the pose: reach boundary in a direction the slide cannot fix"
                     if not any(res < 1e-6 for dx, (res, _) in rows.items() if dx != 0)
                     else "pose held for every x but the arm stays singular: the singular set is invariant under the x self-motion"))
print("=" * 100)

# ------------------------------------------------ write the output above to results/step04d_rescue_by_x.md
sys.stdout = tee.out
out = Path(__file__).resolve().parent.parent / "results" / "step04d_rescue_by_x.md"
out.parent.mkdir(exist_ok=True)
header = "# Step 4D: can the slide rescue an arm singularity?" + chr(10) * 2 + "Script: `scripts/step04d_rescue_by_x.py`" + chr(10) * 2
fence = "```"
out.write_text(header + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(out.parent.parent)}")
