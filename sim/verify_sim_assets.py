"""Check that the simulator URDF has exactly the kinematics of p6r.kinematics
(run from the repo root:  python sim/verify_sim_assets.py)."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from p6r.kinematics import URDFChain, fk_dh  # noqa: E402

sim = URDFChain(str(ROOT / 'sim' / 'umr_ur12e_sim.urdf'))
ref = URDFChain(str(ROOT / 'umr_urdf' / 'umr_ur12e.urdf'))
names = [j['name'] for j in sim.active]
print('movable-joint order seen by the simulators:', names)
assert names == ['base_x_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

rng = np.random.default_rng(0)
Q = np.column_stack([rng.uniform(-1.5, 1.5, 5000), rng.uniform(-np.pi, np.pi, (5000, 6))])
Ts, Tr, Td = sim.fk(Q), ref.fk(Q), fk_dh(Q)
print('sim URDF vs project URDF : %.1e' % np.abs(Ts - Tr).max())
print('sim URDF vs D-H FK       : %.1e' % np.abs(Ts - Td).max())

for line in (ROOT / 'sim' / 'umr_ur12e_sim.urdf').read_text().splitlines():
    if 'filename=' in line:
        f = line.split('filename="')[1].split('"')[0]
        assert (ROOT / 'sim' / f).exists(), f'missing mesh {f}'
print('all referenced meshes exist')
