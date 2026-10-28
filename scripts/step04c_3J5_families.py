"""Step 4C: 3J5 across the singular families: which block sees which family.

    3J5 = blockdiag(R3^T, R3^T) @ 0J_O,   0J_O = arm Jacobian with the velocity point O on the wrist

The UR12e wrist is offset: axes 4, 5 meet at O4 and axes 5, 6 meet at O5, 0.11985 m apart.  No point lies
on all three axes, so the course's block-triangular form [A 0; B W] is never exact here; one column leaks
into C.  The script shows, for the report:
  1. O5 from the DH model is the wrist_2 origin of the URDF walk (two models, same point).
  2. The block form [A C; B W] at a regular q, with the leak column of length d5 = 0.11985 m.
  3. det 3J5 = det 0J_red at every q, while det A * det W is only an approximation.
  4. Which block sees which family:  det W -> wrist (q5 = 0),  det A at O4 -> elbow (q3 = 0, pi),
     det A at O5 -> shoulder.  det A = 0 alone is neither necessary nor sufficient for a singularity.

Run from the repo root:   python scripts/step04c_3J5_families.py
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jacobian_p6r import jacobian_reduced, jacobian_3J5, blocks_3J5, dh_frame, joint_axes, print_J, NAMES

np.set_printoptions(precision=4, suppress=True, linewidth=140)
pi = np.pi
det = np.linalg.det
smin = lambda J: np.linalg.svd(J, compute_uv=False)[-1]
to_q = lambda c: np.r_[c[0], np.deg2rad(c[1:])]


def find_shoulder_singularity(q1=0.0, q2=55.0, q3=90.0, q5=-90.0, q6=0.0, lo=110.0, hi=114.0):
    """Bisection on q4 [deg] for det J_red = 0 with the elbow and wrist away from their own families."""
    f = lambda q4: det(jacobian_reduced(to_q([0.5, q1, q2, q3, q4, q5, q6])))
    a, b = lo, hi
    assert f(a) * f(b) < 0, "no sign change: adjust the bracket"
    for _ in range(60):
        m = 0.5 * (a + b)
        a, b = (a, m) if f(a) * f(m) < 0 else (m, b)
    return [0.5, q1, q2, q3, 0.5 * (a + b), q5, q6]


CASES = [
    ("regular",                               [0.5, 10, -60, 90, -120, -90, 30]),
    ("wrist      q5 = 0",                     [0.5, 10, -60, 90, -120,   0, 30]),
    ("elbow      q3 = 0",                     [0.5,  0, -45,  0,  -45, -90,  0]),
    ("elbow      q3 = 180",                   [0.5,  0, -45, 180, -45, -90,  0]),
    ("shoulder   (q4 by bisection)",          find_shoulder_singularity()),
    ("q = 0      (elbow + wrist)",            [0.5,  0,   0,  0,    0,   0,  0]),
    ("det A(O5) ~ 0 but regular",             [0.5, 99,  -4,  6,  114,  83, 82]),
]

# ---------------------------------------------------------------- 1. the point O5 and the frame 3
q = to_q(CASES[0][1])
O5_dh = dh_frame(q, 5)[:3, 3]
axes, _, _ = joint_axes(q)
O5_urdf = axes[5][2]                                  # axes[k] = (type, z_k, p_k); k = 5 is wrist_2
print("=== velocity point O5 (intersection of axes 5 and 6) ===")
print("DH frame 5 origin  :", O5_dh)
print("URDF wrist_2 origin:", O5_urdf, f"   |diff| = {np.linalg.norm(O5_dh - O5_urdf):.1e}")
z5, z6 = axes[5][1], axes[6][1]
print(f"O5 on axis 5 and on axis 6: |z5 x (O5 - p5)| = {np.linalg.norm(np.cross(z5, O5_dh - axes[5][2])):.1e},"
      f"  |z6 x (O5 - p6)| = {np.linalg.norm(np.cross(z6, O5_dh - axes[6][2])):.1e}")
O4_dh = dh_frame(q, 4)[:3, 3]
print(f"O4 (axes 4, 5) to O5 (axes 5, 6): |O5 - O4| = {np.linalg.norm(O5_dh - O4_dh):.5f} m = d5  (offset wrist)")
print("R3 (columns = DH frame-3 axes in the world):\n", dh_frame(q, 3)[:3, :3])

# ---------------------------------------------------------------- 2. the block structure at a regular q
J35 = jacobian_3J5(q)
b = blocks_3J5(J35)
print(f"\n=== 3J5 (point O5) at x = {q[0]} m, q = {CASES[0][1][1:]} deg   rows [v; w] in DH frame 3, columns q1..q6 ===")
print_J(J35, NAMES[1:])
print("\nA (q1..q3 -> v of O5):\n", b["A"])
print("C (q4..q6 -> v of O5), the offset-wrist leak:\n", b["C"])
print("B (q1..q3 -> w):\n", b["B"])
print("W (q4..q6 -> w):\n", b["W"])
print(f"\n|C| column norms, point O5: {np.linalg.norm(b['C'], axis=0)}   (column 4 = d5, columns 5, 6 = 0)")
print(f"|C| column norms, point O4: {np.linalg.norm(blocks_3J5(jacobian_3J5(q, 'O4'))['C'], axis=0)}   (column 6 = d5)")

# ---------------------------------------------------------------- 3. which block sees which family
print("\n=== determinants per family (det 3J5 is the same for both points and equals det 0J_red) ===")
print(f"{'configuration':30s} {'q [deg]':34s} {'det 0J_red':>11s} {'det 3J5':>11s} {'det W':>9s} {'detA(O4)':>9s} {'detA(O5)':>9s} {'detA*detW(O5)':>14s} {'rank':>5s}")
for name, c in CASES:
    qq = to_q(c)
    Jr, J5, J4 = jacobian_reduced(qq), jacobian_3J5(qq, "O5"), jacobian_3J5(qq, "O4")
    b5, b4 = blocks_3J5(J5), blocks_3J5(J4)
    qs = np.array2string(np.rad2deg(qq[1:]), precision=1, suppress_small=True)
    print(f"{name:30s} {qs:34s} {det(Jr):11.2e} {det(J5):11.2e} {det(b5['W']):9.2e} {det(b4['A']):9.2e} "
          f"{det(b5['A']):9.2e} {det(b5['A']) * det(b5['W']):14.2e} {np.linalg.matrix_rank(J5, tol=1e-9):5d}")

rng = np.random.default_rng(0)
Q = np.c_[rng.uniform(-1.5, 1.5, 200), rng.uniform(-pi, pi, (200, 6))]
gap = max(abs(det(jacobian_3J5(qq)) - det(jacobian_reduced(qq))) for qq in Q)
rel = []
for qq in Q:
    J5 = jacobian_3J5(qq); bb = blocks_3J5(J5)
    rel.append(abs(det(J5) - det(bb["A"]) * det(bb["W"])) / max(abs(det(J5)), 1e-12))
print(f"\n200 random q:  max |det 3J5 - det 0J_red| = {gap:.1e}")
print(f"               |det 3J5 - det A det W| / |det 3J5| (point O5): median {np.median(rel):.2f}, max {np.max(rel):.1f}")

print("""
reading:
  det 3J5 = det A * det(W - B A^-1 C).  With C = 0 (intersecting wrist) this is det A * det W and each block
  owns its families.  The UR12e leak C has one column of length d5 = 0.11985 m, so:
    - W still isolates the wrist family exactly: det W = 0 <-> q5 = 0 (W does not depend on the point).
    - the elbow family (q3 = 0, pi) is det A = 0 only with the point at O4;  at O5 det A stays finite.
    - the shoulder family is det A = 0 only with the point at O5;  at O4 det A stays finite.
    - det A = 0 does not imply a singularity (last row: det A(O5) = -1e-4, det 3J5 = -0.04, rank 6).
  So the course's block picture names the three families but no longer certifies them; the certificate is
  sigma_min or rank of the full 6 x 6.  And a wrist-singular lost direction is a screw, not a pure rotation,
  which is why the slide can sometimes rescue it (step 4A).""")
