"""Step 4B: for one configuration, draw the course subchain Jacobian 3J5 of the arm and test
                det 3J5  =?  det A * det W

    3J5 = blockdiag(R3^T, R3^T) @ 0J_O      velocity point O on the wrist, expressed in UR DH frame 3
    blocks:   3J5 = [ A  C ]     A: q1..q3 -> v of O      C: q4..q6 -> v of O   (0 for an intersecting wrist)
                    [ B  W ]     B: q1..q3 -> w           W: q4..q6 -> w

Run from the repo root:   python scripts/step04b_3J5.py
                          python scripts/step04b_3J5.py 0.5 10 -60 90 -120 -90 30   (x [m], q1..q6 [deg])
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jacobian_p6r import jacobian_reduced, jacobian_3J5, blocks_3J5

det = np.linalg.det

# ------------------------------------------------ input: one configuration (x in m, arm in deg)
x_m = 0.5
q_deg = [10, -60, 90, -120, -90, 30]    # regular
# q_deg = [10, -60, 90, -120, 0, 30]    # wrist singular, q5 = 0
# q_deg = [0, -45, 0, -45, -90, 0]      # elbow singular, q3 = 0
POINT = "O5"                            # "O5": axes 5, 6 meet there   "O4": axes 4, 5 meet there
# -----------------------------------------------------------------------------------------------
if len(sys.argv) == 8:
    x_m, q_deg = float(sys.argv[1]), [float(v) for v in sys.argv[2:]]
q = np.r_[x_m, np.deg2rad(q_deg)]

J = jacobian_3J5(q, POINT)
b = blocks_3J5(J)

# ------------------------------------------------ 1. draw 3J5 with the four blocks separated
print(f"x = {x_m} m,  arm q = {q_deg} deg,  velocity point {POINT},  frame = UR DH frame 3\n")
cols = ["q1", "q2", "q3", "q4", "q5", "q6"]
rows = ["vx", "vy", "vz", "wx", "wy", "wz"]
print("3J5 =")
print("        " + "".join(f"{c:>9s}" for c in cols[:3]) + "   |" + "".join(f"{c:>9s}" for c in cols[3:]))
for i, (lbl, row) in enumerate(zip(rows, J)):
    if i == 3:
        print("       " + "-" * 27 + "--+" + "-" * 27)
    print(f"  {lbl:4s}  " + "".join(f"{v:9.4f}" for v in row[:3]) + "   |" + "".join(f"{v:9.4f}" for v in row[3:]))
print("\n           [   A   |   C   ]")
print("           [   B   |   W   ]")

# ------------------------------------------------ 2. determinants
dA, dW, dJ = det(b["A"]), det(b["W"]), det(J)
print(f"\ndet A           = {dA:9.4f}      (A = 3J5[0:3, 0:3], how q1..q3 move {POINT})")
print(f"det W           = {dW:9.4f}      (W = 3J5[3:6, 3:6], how q4..q6 rotate the tool)")
print(f"det A * det W   = {dA * dW:9.4f}")
print(f"det 3J5         = {dJ:9.4f}")
print(f"det 0J_red      = {det(jacobian_reduced(q)):9.4f}      (same as det 3J5: point shift and rotation have unit det)")

# ------------------------------------------------ 3. why they differ: the leak block C
cn = np.linalg.norm(b["C"], axis=0)
print(f"\nC = 3J5[0:3, 3:6] column norms = {np.array2string(cn, precision=4)}")
if np.allclose(cn, 0, atol=1e-9):
    print("C = 0  ->  3J5 is block triangular and det 3J5 = det A * det W exactly.")
else:
    k = int(np.argmax(cn)) + 4
    print(f"C is not 0: column q{k} still moves {POINT} by {cn.max():.4f} m per rad (= d5, the wrist offset).")
    print("So 3J5 is NOT block triangular and det 3J5 = det A * det(W - B A^-1 C), not det A * det W.")
    if abs(dA) > 1e-12:
        S = b["W"] - b["B"] @ np.linalg.solve(b["A"], b["C"])
        print(f"check:  det A * det(W - B A^-1 C) = {dA * det(S):9.4f}")
