"""Forward kinematics of the UR12e by the Product of Exponentials (lecture 2).

Space form:
    T_world_flange(q) = exp([S1] q1) @ exp([S2] q2) @ ... @ exp([S6] q6) @ M
Each screw S_i = (w_i, v_i) is joint i's axis at the home configuration q = 0, in the world frame:
    w_i = unit direction of the axis,   v_i = -w_i x r_i,   r_i = any point on the axis.
M is the flange pose at q = 0.  The six (w_i, r_i) below are read off the step-1 axis table.

Run from the repo root:
    python scripts/fk_poe.py                       -> q = 0, comparison with fk_urdf
    python scripts/fk_poe.py 10 -60 90 -120 -90 30 -> q in degrees
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fk_urdf import fk as fk_urdf, CHAIN, origin   # reference model

np.set_printoptions(precision=5, suppress=True)

# ================================ SCREW AXES (home configuration) ================================
# (joint, direction w, point on axis r)  in the world frame at q = 0   -- from step 1
AXES = [
    ("1 shoulder_pan",  [0, 0,  1], [0,       0,       0.1807 ]),
    ("2 shoulder_lift", [0, 1,  0], [0,       0,       0.1807 ]),
    ("3 elbow",         [0, 1,  0], [0.6127,  0,       0.1807 ]),
    ("4 wrist_1",       [0, 1,  0], [1.18425, 0.17415, 0.1807 ]),
    ("5 wrist_2",       [0, 0, -1], [1.18425, 0.17415, 0.06085]),
    ("6 wrist_3",       [0, 1,  0], [1.18425, 0.2907,  0.06085]),
]
# home pose of the flange (q = 0), from step 1
M = np.array([[0, -1, 0, 1.18425],
              [1,  0, 0, 0.2907 ],
              [0,  0, 1, 0.06085],
              [0,  0, 0, 1      ]], float)
# ==================================================================================================


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def screw(w, r):
    """Revolute screw from axis direction w and point r:  (w, v) with v = -w x r."""
    w = np.asarray(w, float) / np.linalg.norm(w)
    return w, -np.cross(w, np.asarray(r, float))


def exp_twist(w, v, q):
    """exp([S] q) for a unit-rotation screw (Rodrigues + translation term), closed form."""
    K = skew(w)
    R = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    G = np.eye(3) * q + (1 - np.cos(q)) * K + (q - np.sin(q)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = G @ v
    return T


SCREWS = [screw(w, r) for _, w, r in AXES]


def fk(q, verbose=False):
    """T_world_flange by the product of exponentials."""
    T = np.eye(4)
    for i, (w, v) in enumerate(SCREWS):
        T = T @ exp_twist(w, v, q[i])
        if verbose:
            print(f"after exp(S{i + 1} q{i + 1}):  moved flange = {(T @ M)[:3, 3]}")
    return T @ M


# ---------------------------------------------------------------- self-check of the inputs
def axes_from_urdf():
    """Recompute (w_i, r_i) and M by chaining the URDF at q = 0 (independent of the table above)."""
    T, out = np.eye(4), []
    for name, xyz, rpy, axis in CHAIN:
        T = T @ origin(xyz, rpy)
        if axis is not None:
            out.append((T[:3, :3] @ np.asarray(axis, float), T[:3, 3].copy()))
    return out, T


if __name__ == "__main__":
    print("=== screw axes S_i = (w_i, v_i) ===")
    for (name, _, _), (w, v) in zip(AXES, SCREWS):
        print(f"{name:18s} w = {w}   v = {v}")

    axes_ref, M_ref = axes_from_urdf()
    err_w = max(np.linalg.norm(np.cross(wr, screw(a[1], a[2])[0])) for (wr, _), a in zip(axes_ref, AXES))
    err_r = max(np.linalg.norm(np.cross(np.asarray(a[2], float) - rr, wr)) for (wr, rr), a in zip(axes_ref, AXES))
    print(f"\ntable vs URDF at q = 0:  axis direction mismatch {err_w:.1e}, point-off-axis {err_r:.1e} m, "
          f"M mismatch {np.abs(M - M_ref).max():.1e}")

    q = np.deg2rad([float(v) for v in sys.argv[1:]]) if len(sys.argv) == 7 else np.zeros(6)
    print("\nq [deg] =", np.rad2deg(q), "\n")
    T_poe = fk(q, verbose=True)
    T_ref = fk_urdf(q)
    print("\nT_world_flange (PoE) =");  print(T_poe)
    print("\nT_world_flange (URDF) ="); print(T_ref)
    e_p = np.linalg.norm(T_poe[:3, 3] - T_ref[:3, 3])
    c = (np.trace(T_ref[:3, :3].T @ T_poe[:3, :3]) - 1) / 2
    print(f"\nposition error    = {e_p:.3e} m")
    print(f"orientation error = {np.degrees(np.arccos(np.clip(c, -1, 1))):.3e} deg")

    rng = np.random.default_rng(0)
    worst = max(np.abs(fk(qq) - fk_urdf(qq)).max() for qq in rng.uniform(-np.pi, np.pi, (100, 6)))
    print(f"max |T_poe - T_urdf| over 100 random q = {worst:.3e}")
