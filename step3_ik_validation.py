"""Step 3: analytical IK - round trip, labels, persistence, speed, visualisation."""
import time
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from p6r.kinematics import *
from p6r.ik import *

rng = np.random.default_rng(2)

# ---------------------------------------------------------- (a) round trip
N = 200_000
q_true = rng.uniform(-np.pi, np.pi, (N, 6))
t0 = time.perf_counter()
qs, st = ik_ur(fk_ur(q_true))
dt = time.perf_counter() - t0
dq = np.abs(wrap(qs - q_true[:, None])).max(2)               # (N, 8)
found = np.nanmin(np.where(st == OK, dq, np.inf), 1)
s5 = np.abs(np.sin(q_true[:, 4]))
reg = s5 > 1e-3
print(f'{N} poses solved in {dt:.2f} s  ({N / dt / 1e6:.2f} M poses/s, 8 branches each)')
print(f'original q recovered (|dq|<1e-6) : {np.mean(found[reg] < 1e-6) * 100:.4f} % of regular-wrist samples')
print(f'worst recovery error              : {found[reg].max():.2e} rad')
print(f'every OK root passes FK check     : {(st != FK_FAIL).all()}')
print(f'solutions per pose (OK)           : ' + str(np.bincount((st == OK).sum(1), minlength=9)))

# label check: the branch that reproduces q_true carries q_true's own label
b_hit = np.nanargmin(np.where(st == OK, dq, np.inf), 1)
lab_ok = (BRANCH_SIGNS[b_hit] == branch_of(q_true)).all(1)
print(f'branch label of recovered root matches label of q_true: {lab_ok[reg].mean() * 100:.3f} %')

# ---------------------------------------------------------- (b) label persistence
# random smooth joint paths -> FK -> IK; follow each branch by label and check
# that the same-label root is also the nearest root to the previous step.
n_paths, n_steps, viol, tot = 300, 400, 0, 0
for _ in range(n_paths):
    q0, q1 = rng.uniform(-2.5, 2.5, 6), rng.uniform(-2.5, 2.5, 6)
    path = q0 + np.linspace(0, 1, n_steps)[:, None] * (q1 - q0)
    Qp, Sp = ik_ur(fk_ur(path))
    for b in range(8):
        for k in range(1, n_steps):
            if Sp[k, b] != OK or Sp[k - 1, b] != OK:
                continue
            d = np.linalg.norm(wrap(Qp[k] - Qp[k - 1, b]), axis=1)
            d[Sp[k] != OK] = np.inf
            tot += 1
            viol += np.argmin(d) != b
print(f'label persistence: nearest neighbour == same label in {100 * (1 - viol / tot):.3f} % of {tot} steps')

# ---------------------------------------------------------- (c) one tool pose, all solutions, x = 0
R_down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.]])          # tool z pointing down
T_W = np.eye(4); T_W[:3, :3] = R_down; T_W[:3, 3] = [0.55, 0.35, 0.60]
qW, sW = ik_world(T_W, 0.0)
print('\nRepresentative pose p = (0.55, 0.35, 0.60) m, tool down, x = 0')
for b in range(8):
    if sW[0, b] == OK:
        Qb = np.r_[0.0, qW[0, b]]
        e = np.linalg.norm(fk_dh(Qb)[:3, 3] - T_W[:3, 3])
        print(f'  {BRANCH_LABELS[b]}: q = {np.round(np.degrees(qW[0, b]), 1)} deg   '
              f'sig_min(J_red) = {sigma_min(jacobian_reduced(Qb)):.3f}   |dp| = {e:.1e}')

fig = plt.figure(figsize=(16, 8))
for b in range(8):
    ax = fig.add_subplot(2, 4, b + 1, projection='3d')
    if sW[0, b] == OK:
        F = dh_frames(np.r_[0.0, qW[0, b]])
        P = F[1:, :3, 3]
        P = np.vstack([[0, 0, H_MOUNT], P[1:]])
        ax.plot(*P.T, '-o', lw=3, ms=4, color='C0')
        ax.scatter(*T_W[:3, 3], color='r', s=40)
        ax.quiver(*T_W[:3, 3], *(0.15 * T_W[:3, 2]), color='r')
    # base box
    for zz in (0, H_MOUNT):
        bx = np.array([[-.415, -.27], [.415, -.27], [.415, .27], [-.415, .27], [-.415, -.27]])
        ax.plot(bx[:, 0], bx[:, 1], zz, color='gray', lw=1)
    ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(0, 1.6)
    ax.set_box_aspect((1, 1, 0.8)); ax.view_init(22, -60)
    ax.set_title(BRANCH_LABELS[b] + ('' if sW[0, b] == OK else '  (none)'), fontsize=10)
    ax.tick_params(labelsize=6)
plt.suptitle('All analytical IK solutions, tool-down pose at (0.55, 0.35, 0.60) m, base at x = 0')
plt.tight_layout(); plt.savefig('figures/step3_all_ik_solutions.png', dpi=130)
print('saved figures/step3_all_ik_solutions.png')
