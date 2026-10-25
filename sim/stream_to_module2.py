"""Stream a joint path live to Module 2 ("Joint commands & FK") over WebSocket.

    pip install websockets
    python sim/stream_to_module2.py [joint_csv] [--dt 0.05] [--loop]

1. Open https://epfl-lasa.github.io/eng-654/modules/module2/ and load
   sim/umr_ur12e_sim.urdf + all files in sim/meshes/ (select them together).
2. Run this script (it waits for the page).
3. In the page: 'Python / WebSocket' -> Connect.

Default joint file: sim/data/m5_joint_moving_base.csv (run sim/export_examples.py first).
Replace the CSV by any (m, 7) array from your own code, e.g. stream_joint_path(Q_planned).

Note: Chrome/Edge allow an https page to connect to ws://localhost. If the
connection is refused, check that nothing else uses port 8765 and that a firewall
does not block local connections.
"""
import argparse
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from p6r.sim_io import stream_joint_path  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('csv', nargs='?', default=Path(__file__).resolve().parent / 'data' / 'm5_joint_moving_base.csv')
ap.add_argument('--dt', type=float, default=0.05)
ap.add_argument('--loop', action='store_true')
a = ap.parse_args()
Q = np.loadtxt(a.csv, delimiter=',', skiprows=1)
stream_joint_path(Q, dt=a.dt, loop=a.loop)
