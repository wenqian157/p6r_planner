"""Step 1: whole-body model.  URDF vs D-H vs PoE forward kinematics."""
import numpy as np
from p6r.kinematics import *

np.set_printoptions(precision=5, suppress=True)
urdf = URDFChain('umr_ur12e.urdf')
print('Active joints:', [j['name'] for j in urdf.active])

# --- home pose and screws
print('\nHome pose M (tool0 in world, Q = 0):\n', M_HOME)
print('\nScrew axes S_i = [omega; v] at home (columns = x, q1..q6):\n', S_HOME)
for i in range(7):
    w, v = S_HOME[:3, i], S_HOME[3:, i]
    if np.allclose(w, 0):
        print(f'  S{i}: prismatic, direction {v}')
    else:   # a point on the axis: p = w x v  (for |w| = 1)
        print(f'  S{i}: revolute, omega {w}, point on axis {np.cross(w, v)}')

# --- random agreement test
rng = np.random.default_rng(0)
N = 20000
Q = np.column_stack([rng.uniform(-1.5, 1.5, N), rng.uniform(-np.pi, np.pi, (N, 6))])
T_u, T_d, T_p = urdf.fk(Q), fk_dh(Q), fk_poe(Q)


def err(Ta, Tb):
    dp = np.linalg.norm(Ta[:, :3, 3] - Tb[:, :3, 3], axis=1)
    dR = np.linalg.norm(Ta[:, :3, :3] - Tb[:, :3, :3], axis=(1, 2))
    return dp.max(), dR.max()


for name, (a, b) in {'URDF vs DH ': (T_u, T_d), 'URDF vs PoE': (T_u, T_p),
                     'DH   vs PoE': (T_d, T_p)}.items():
    print(f'{name}: max |dp| = {err(a, b)[0]:.2e} m, max |dR|_F = {err(a, b)[1]:.2e}')

# --- the redundancy statement: fixing x only shifts the arm
qa = Q[:, 1:]
T_comp = T_world_urbase(Q[:, 0]) @ fk_ur(qa)
print('World FK  ==  T_world_urbase(x) @ UR FK :  max err %.2e' % err(T_d, T_comp)[0])

# --- URDF intermediate frames vs D-H frames (base frame and flange frame)
fr = urdf.fk(np.zeros(7), frames=True)
print('\nURDF base_link_inertia (= UR base) frame at Q=0 (should be world-aligned, z=0.41):\n', fr['base_link-base_link_inertia'][0])
