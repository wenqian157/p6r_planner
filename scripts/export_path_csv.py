"""Export a workspace path as CSV for the course visualiser (module 5).

Format: seven columns  x, y, z, qw, qx, qy, qz  (one end-effector pose per row), with a header row.
    position   metres, world frame (the rail frame: x along the rail, z up, origin on the floor under x = 0)
    quaternion unit, scalar first (qw, qx, qy, qz), rotation of the end-effector link in the world frame
The path is the rectangle of steps 7 / 7C, sampled at 221 values of s (the 111 samples of the maps plus their midpoints), so every second row lines up with
the (s, x) maps and with any joint path exported from the planner.

EE_LINK chooses which URDF link the poses refer to:
    "flange"  the frame used everywhere in this project (x = outward normal)
    "tool0"   the UR tool frame (z = outward normal), = flange * rpy(pi/2, 0, pi/2)

Run from the repo root:
    python scripts/export_path_csv.py              horizontal rectangle
    python scripts/export_path_csv.py 30 20        plane tilted 30 deg about x and 20 deg about y
    python scripts/export_path_csv.py 30 20 tool0  same, poses of tool0
Output: results/paths/workspace_path_<tag>_<link>.csv
"""
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent

TILT_X, TILT_Y, EE_LINK = 0.0, 0.0, "flange"
args = sys.argv[1:]
if len(args) >= 2:
    TILT_X, TILT_Y = float(args[0]), float(args[1])
if len(args) == 3:
    EE_LINK = args[2]

# ------------------------------------------------ same path definition as step07c_sx_tilted.py
PIVOT = np.array([0.0, 0.25, 0.9])
UV = np.array([[-0.8, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6]])
R_FLAT = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)          # flange: x = tool axis, pointing down
N_SAMPLES = 221                                          # 221 = every sample of the 111-point maps plus the midpoints
S = np.linspace(0, 1, N_SAMPLES)
cx, sx_, cy, sy_ = (f(np.deg2rad(a)) for a in (TILT_X, TILT_Y) for f in (np.cos, np.sin))
R_TILT = np.array([[1, 0, 0], [0, cx, -sx_], [0, sx_, cx]]) @ np.array([[cy, 0, -sy_], [0, 1, 0], [sy_, 0, cy]])
SEG = np.roll(UV, -1, axis=0) - UV
CUM = np.r_[0, np.cumsum(np.linalg.norm(SEG, axis=1))] / np.linalg.norm(SEG, axis=1).sum()


def rpy(r, p, y):
    cr, sr, cp, sp, cy_, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


R_EE = R_TILT @ R_FLAT @ (rpy(np.pi / 2, 0, np.pi / 2) if EE_LINK == "tool0" else np.eye(3))


def quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix, w >= 0."""
    t = np.trace(R)
    if t > 0:
        w = 0.5 * np.sqrt(1 + t)
        q = np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = np.zeros(4)
        q[1 + i] = 0.5 * r
        q[0] = (R[k, j] - R[j, k]) / (2 * r)
        q[1 + j] = (R[j, i] + R[i, j]) / (2 * r)
        q[1 + k] = (R[k, i] + R[i, k]) / (2 * r)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def position(s):
    k = min(np.searchsorted(CUM, s, side="right") - 1, 3)
    uv = UV[k] + (s - CUM[k]) / (CUM[k + 1] - CUM[k]) * SEG[k]
    return PIVOT + R_TILT @ np.array([uv[0], uv[1], 0.0])


q = quat_wxyz(R_EE)
rows = np.array([np.r_[position(s), q] for s in S])

# check: the quaternion reproduces the rotation matrix
w, x, y, z = q
R_back = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
assert np.abs(R_back - R_EE).max() < 1e-12

tag = ("".join(f"{n}{a:g}" for a, n in ((TILT_X, "x"), (TILT_Y, "y")) if a) or "flat")
out = HERE.parent / "results" / "paths" / f"workspace_path_{tag}_{EE_LINK}.csv"
out.parent.mkdir(parents=True, exist_ok=True)
np.savetxt(out, rows, delimiter=",", header="x,y,z,qw,qx,qy,qz", comments="", fmt="%.6f")

print(f"written {out.relative_to(HERE.parent)}: {len(rows)} rows, columns x,y,z,qw,qx,qy,qz")
print(f"  path: rectangle tilted {TILT_X:g} deg about x and {TILT_Y:g} deg about y, link = {EE_LINK}, closed (first row = last row)")
print(f"  position range [m]: x {rows[:, 0].min():+.3f}..{rows[:, 0].max():+.3f}, y {rows[:, 1].min():+.3f}..{rows[:, 1].max():+.3f}, "
      f"z {rows[:, 2].min():+.3f}..{rows[:, 2].max():+.3f}")
print(f"  quaternion (qw, qx, qy, qz) = {np.round(q, 6)}  (constant along the path)")
print(f"  {EE_LINK} axes in the world (columns x, y, z):\n{np.round(R_EE, 4)}")
