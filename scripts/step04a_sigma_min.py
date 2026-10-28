"""Step 4A: for one configuration, print J (6x7), J_red (6x6), and the two sigma_min values.

    J     = full geometric Jacobian of the P6R, columns (x, q1..q6)
    J_red = J with the x column removed = Jacobian of the arm alone (x fixed)
    sigma_min = smallest singular value; 0 means singular.

Run from the repo root:   python scripts/step04a_sigma_min.py
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jacobian_p6r import jacobian, print_J, NAMES

np.set_printoptions(precision=4, suppress=True)

# ------------------------------------------------ input: one configuration (x in m, arm in deg)
x_m = 0.5
q_deg = [0, -45, 0, -45, -90, 0]        # elbow straight (q3 = 0), arm plane = x-z
# q_deg = [90, -45, 0, -45, -90, 0]     # same, arm plane = y-z
# q_deg = [0, -90, 0, 0, -90, 0]        # arm vertical
# q_deg = [10, -60, 90, -120, -90, 30]  # regular
q = np.r_[x_m, np.deg2rad(q_deg)]
# -----------------------------------------------------------------------------------------------

J = jacobian(q)          # 6 x 7
J_red = J[:, 1:]         # 6 x 6

print(f"x = {x_m} m,  arm q = {q_deg} deg\n")
print("J  (6 x 7), columns (x, q1..q6):")
print_J(J, NAMES)
print("\nJ_red  (6 x 6), columns (q1..q6):")
print_J(J_red, NAMES[1:])

# singular values: s is sorted largest -> smallest, so s[-1] is sigma_min
U_red, s_red, _ = np.linalg.svd(J_red)
s_full = np.linalg.svd(J, compute_uv=False)

print("\nsingular values of J_red:", s_red)
print("singular values of J    :", s_full)
print(f"\nsigma_min(J_red) = {s_red[-1]:.4e}")
print(f"sigma_min(J)     = {s_full[-1]:.4e}")

# the lost direction: column of U paired with sigma_min(J_red)
u = U_red[:, -1]
print("\nlost task direction u = U_red[:, -1] =", u, "  [vx vy vz wx wy wz]")
print(f"u_x = {abs(u[0]):.4f}  ->", "the slide can supply this direction" if abs(u[0]) > 1e-9
      else "no x component: the slide cannot help")
