"""Symbolic Jacobians of the P6R with sympy, lecture-5 style: 0J6 (= J_red) and the full 6 x 7 J.

Built from the DH rows with symbolic joint angles q1..q6 and symbolic lengths d1, a2, a3, d4, d5, d6.
    1.  0J6     6 x 6 arm Jacobian in DH frame 0, velocity point = flange (origin of frame 6).
                This is J_red (x fixed) up to the constant rotation R0 between the world and DH frame 0.
    2.  J       6 x 7 whole-body Jacobian in the world frame: slide column e = [1 0 0 0 0 0]^T, then
                blockdiag(R0, R0) 0J6 with R0 = Rz(pi); the mount translation does not enter a Jacobian.
    3.  det J_red factored: the singular set of the arm as an equation.
Every symbolic matrix is checked numerically against jacobian_p6r.py on random configurations.

Run from the repo root:   python scripts/jacobian_symbolic.py      (about 80 s, the determinant is the slow part)
Output printed and written to results/jacobian_symbolic.md
"""
import sys
import time
from pathlib import Path
import numpy as np
import sympy as sp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fk_dh import DH as DH_NUM
from jacobian_p6r import jacobian, jacobian_reduced


class Tee:
    """Print to the terminal and keep a copy, so the output can be written to results/ as is."""
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
t_start = time.time()
simp = lambda e: sp.factor(sp.trigsimp(sp.expand(e)))

# ---------------------------------------------------------------- symbols and DH rows
q = sp.symbols("q1:7", real=True)
d1, a2, a3, d4, d5, d6 = sp.symbols("d1 a2 a3 d4 d5 d6", real=True)
DH_SYM = [(d1, 0, sp.pi / 2), (0, a2, 0), (0, a3, 0), (d4, 0, sp.pi / 2), (d5, 0, -sp.pi / 2), (d6, 0, 0)]
LENGTHS = dict(zip((d1, a2, a3, d4, d5, d6),
                   (DH_NUM[0][0], DH_NUM[1][1], DH_NUM[2][1], DH_NUM[3][0], DH_NUM[4][0], DH_NUM[5][0])))


def dh_row_sym(theta, d, a, alpha):
    c, s, ca, sa = sp.cos(theta), sp.sin(theta), sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([[c, -s * ca, s * sa, a * c],
                      [s, c * ca, -c * sa, a * s],
                      [0, sa, ca, d],
                      [0, 0, 0, 1]])


T = [sp.eye(4)]                                   # T[k] = pose of DH frame k in frame 0
for qi, (d, a, alpha) in zip(q, DH_SYM):
    T.append(T[-1] * dh_row_sym(qi, d, a, alpha))

# ================================================================ 1. 0J6: frame 0, point = flange
p6 = T[6][:3, 3]
J06 = sp.zeros(6, 6)
for k in range(6):                                # joint k+1 turns about z of frame k, through its origin
    z, p = T[k][:3, 2], T[k][:3, 3]
    J06[:3, k] = z.cross(p6 - p)
    J06[3:, k] = z
J06 = J06.applyfunc(lambda e: sp.trigsimp(sp.expand(e)))

def print_columns(J, names):
    for k, n in enumerate(names):
        print(f"  column {n}:")
        for lbl, e in zip(["vx", "vy", "vz", "wx", "wy", "wz"], J[:, k]):
            print(f"     {lbl}: {e}")


QN = [f"q{k}" for k in range(1, 7)]
print("=== 1. 0J6: arm Jacobian in DH frame 0 of the arm, velocity point = flange, rows [vx vy vz wx wy wz] ===")
print_columns(J06, QN)
print(f"\n  free symbols: {sorted(map(str, J06.free_symbols))}")
print("  -> q6 absent: the last joint moves no axis and no point before the flange.")
print("  -> d1 absent: it only lifts frame 0 and the flange together.")
print("  -> q1 appears in every column: it rotates all columns together, so det does not depend on it.")

# ================================================================ 2. whole-body 6 x 7 in the world frame
R0 = sp.Matrix([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])        # Rz(pi): DH frame 0 = base_link_inertia, yaw pi
e_slide = sp.Matrix([1, 0, 0, 0, 0, 0])
J_full = sp.Matrix.hstack(e_slide, sp.diag(R0, R0) * J06)
J_W6 = sp.diag(R0, R0) * J06
print("\n=== 2. WJ6 = J_red: the same arm Jacobian in the world frame, velocity point = flange ===")
print("  WJ6 = blockdiag(R0, R0) 0J6,  R0 = Rz(pi) = rotation of DH frame 0 in the world (base_link_inertia yaw)")
print("  -> every column has its rows vx, vy, wx, wy negated; vz, wz unchanged.")
print_columns(J_W6, QN)

print("\n=== 3. WJ: whole-body Jacobian (6 x 7) in the world frame, columns (x, q1..q6) ===")
print("  WJ = [ e | WJ6 ],   e = [1 0 0 0 0 0]^T: the slide column is pure x translation and contains no q.")
print("  J_red = WJ[:, 1:] = WJ6 (x fixed).")
print_columns(J_full, ["x"] + QN)

# ================================================================ 3. det J_red
t0 = time.time()
det_red = simp(J06.det(method="berkowitz"))      # det J_red = det 0J6 (det R0 = 1 twice)
print("\n=== 4. det J_red = det 0J6 = det WJ6 (factored) ===")
print("  det J_red =", det_red, f"   [{time.time() - t0:.0f} s]")
print("\n  reading: det J_red = a2 a3 sin(q3) sin(q5) (a2 cos q2 + a3 cos(q2+q3) + d5 sin(q2+q3+q4))")
print("     sin q3 = 0                     elbow family    (q3 = 0, pi)")
print("     sin q5 = 0                     wrist family    (q5 = 0, pi)")
print("     a2 c2 + a3 c23 + d5 s234 = 0   shoulder family (in-plane radial coordinate of O5 is zero)")
print("  det does not contain q1, q6, d1, d4, d6: the singular set lives in (q2, q3, q4, q5), as observed numerically.")
print("  Compare lecture-5 slide 48 (ABB IRB 4600, intersecting wrist):")
print("     det = -a2 (a3 s3 + d4 c3)(a1 + a2 s2 + a3 s_psi + d4 c_psi) s5   -> same wrist factor s5.")

# ================================================================ numeric checks against jacobian_p6r.py
J06_num = sp.lambdify(q, J06.subs(LENGTHS), "numpy")
Jf_num = sp.lambdify(q, J_full.subs(LENGTHS), "numpy")
det_num = sp.lambdify(q, det_red.subs(LENGTHS), "numpy")
R0n = np.array(R0, float)
rng = np.random.default_rng(0)
e06 = efull = edet = 0.0
for _ in range(200):
    qq = rng.uniform(-np.pi, np.pi, 6)
    q7 = np.r_[rng.uniform(-1.5, 1.5), qq]
    Jr = jacobian_reduced(q7)                                           # numeric: world frame, flange point
    e06 = max(e06, np.abs(np.kron(np.eye(2), R0n) @ np.array(J06_num(*qq), float) - Jr).max())
    efull = max(efull, np.abs(np.array(Jf_num(*qq), float) - jacobian(q7)).max())
    edet = max(edet, abs(float(det_num(*qq)) - np.linalg.det(Jr)))
print("\n=== numeric checks, 200 random (x, q), UR12e lengths, against jacobian_p6r.py ===")
print(f"  max |blockdiag(R0,R0) 0J6 - J_red_numeric| = {e06:.1e}")
print(f"  max |J_full - J_numeric (6 x 7)|           = {efull:.1e}")
print(f"  max |det J_red symbolic - numeric|         = {edet:.1e}")
shoulder = [0, 55, 90, 111.95, -90, 0]
f_sh = sp.lambdify(q, (a2 * sp.cos(q[1]) + a3 * sp.cos(q[1] + q[2]) + d5 * sp.sin(q[1] + q[2] + q[3])).subs(LENGTHS))
print(f"  shoulder factor at the step-4C configuration q = {shoulder} deg: {float(f_sh(*np.deg2rad(shoulder))):.1e}")
print(f"\ntotal time {time.time() - t_start:.0f} s")

# ---------------------------------------------------------------- write results/jacobian_symbolic.md
sys.stdout = tee.out
out = HERE.parent / "results" / "jacobian_symbolic.md"
out.parent.mkdir(exist_ok=True)
header = "# Symbolic Jacobians of the P6R: 0J6 (J_red) and the full 6 x 7 J" + chr(10) * 2 + "Script: `scripts/jacobian_symbolic.py`" + chr(10) * 2
fence = "```"
out.write_text(header + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
