"""Generate ready-to-load input files for the ENG-654 simulators (run from the repo root):

    python sim/export_examples.py

sim/data/
  m3_ik_x0.csv                 all IK solutions, tool-down pose at (0.55, 0.35, 0.60), x = 0   (Module 3/4)
  m3_ik_x0.csv.labels.txt      branch label of every row
  m3_ik_multi_x.csv            same world pose, x = -0.5, 0, +0.5 together (base moves too)  (Module 3)
  m5_workspace_path.csv        demo pick-and-place line, tool down, 201 poses              (Module 5)
  m5_joint_fixed_x0.csv        branch S+W-E+ followed with the base frozen at x = 0         (Module 5 / 2)
  m5_joint_moving_base.csv     same branch with the base ramped x(s) = -0.3 + 0.9 s         (Module 5 / 2)

The demo path only shows how the pipeline works. It is NOT the project's planner (steps 4-6).
"""
from pathlib import Path
import numpy as np

from p6r.kinematics import fk_dh, jacobian_reduced, sigma_min, wrap
from p6r.ik import OK, BRANCH_LABELS, ik_world
from p6r.sim_io import (write_ik_solutions_csv, write_joint_csv, write_workspace_csv,
                        check_joint_vs_workspace)

OUT = Path(__file__).resolve().parent / 'data'
OUT.mkdir(exist_ok=True)
R_DOWN = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.]])     # tool z pointing down


def pose(p, R=R_DOWN):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return T


# ------------------------------------------------------------ Module 3: IK multiplicity
T_W = pose([0.55, 0.35, 0.60])
Q0, lab0 = write_ik_solutions_csv(OUT / 'm3_ik_x0.csv', T_W, 0.0)
print(f'Module 3: {len(Q0)} solutions at x = 0 -> {", ".join(lab0)}')

rows, labs = [], []
for x in (-0.5, 0.0, 0.5):
    q, st = ik_world(T_W, x)
    for b in np.flatnonzero(st[0] == OK):
        rows.append(np.r_[x, q[0, b]]); labs.append(f'x={x:+.1f} {BRANCH_LABELS[b]}')
write_joint_csv(OUT / 'm3_ik_multi_x.csv', np.array(rows))
(OUT / 'm3_ik_multi_x.csv.labels.txt').write_text(
    '\n'.join(f'{i:4d}  {l}' for i, l in enumerate(labs, 1)) + '\n')
dp, dR = check_joint_vs_workspace(np.array(rows), np.repeat(T_W[None], len(rows), 0))
print(f'Module 3: {len(rows)} solutions over three base positions, FK check |dp| = {dp:.1e} m')

# ------------------------------------------------------------ Module 5: path + joint paths
s = np.linspace(0, 1, 201)
P = np.column_stack([-0.6 + 1.8 * s, np.full_like(s, 0.45), 0.45 + 0.25 * np.sin(np.pi * s)])
T_path = np.stack([pose(p) for p in P])
write_workspace_csv(OUT / 'm5_workspace_path.csv', T_path)

BRANCH = BRANCH_LABELS.index('S+W-E+')
for name, x in [('m5_joint_fixed_x0.csv', np.zeros_like(s)),
                ('m5_joint_moving_base.csv', -0.3 + 0.9 * s)]:
    q, st = ik_world(T_path, x)
    ok = st[:, BRANCH] == OK
    if not ok.all():
        raise RuntimeError(f'{name}: branch fails at s = {s[~ok][0]:.3f}')
    Q = np.column_stack([x, q[:, BRANCH]])
    write_joint_csv(OUT / name, Q)
    dp, dR = check_joint_vs_workspace(Q, T_path)
    step = np.abs(wrap(np.diff(Q[:, 1:], axis=0))).max()
    print(f'Module 5: {name:26s} FK |dp| = {dp:.1e} m, max joint step {step:.3f} rad, '
          f'min sigma(J_red) = {sigma_min(jacobian_reduced(Q)).min():.3f}')

# which branches survive the whole path at fixed x = 0 vs moving base (for the guide)
for tag, x in [('x = 0      ', np.zeros_like(s)), ('x(s) ramp  ', -0.3 + 0.9 * s)]:
    _, st = ik_world(T_path, x)
    print(f'{tag}: complete branches = {[BRANCH_LABELS[b] for b in range(8) if (st[:, b] == OK).all()]}')
print('files in', OUT)
