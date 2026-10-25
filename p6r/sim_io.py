"""
Bridge between the p6r package and the ENG-654 web simulators
(https://epfl-lasa.github.io/eng-654/modules/moduleN/).

All simulators read the URDF's movable joints in file order. For sim/umr_ur12e_sim.urdf
that order is  Q = [x, q1, q2, q3, q4, q5, q6]  (metres, radians), the same vector used
everywhere in p6r.kinematics, so joint arrays can be written as they are.

    Module 2  joint CSV (playback) or live WebSocket stream   -> write_joint_csv / stream_joint_path
    Module 3  IK-solutions CSV, one 7-value row per solution  -> write_ik_solutions_csv
    Module 4  IK-configurations CSV (row 2 = default config)  -> write_joint_csv
    Module 5  workspace CSV  x,y,z,qw,qx,qy,qz                -> write_workspace_csv
              joint CSV, exactly 7 columns                    -> write_joint_csv
"""
import asyncio
import json
import time

import numpy as np

from .kinematics import fk_dh
from .ik import OK, BRANCH_LABELS, ik_world

JOINT_HEADER = 'x,q1,q2,q3,q4,q5,q6'


# --------------------------------------------------------------------- rotations
def rot_to_quat(R):
    """Rotation matrix/matrices (...,3,3) -> unit quaternion(s) (...,4) as (qw, qx, qy, qz),
    with qw >= 0 (Shepperd's method, robust near 180 deg)."""
    R = np.asarray(R, float)
    single = R.ndim == 2
    R = R.reshape(-1, 3, 3)
    q = np.empty((len(R), 4))
    for k, M in enumerate(R):
        tr = np.trace(M)
        i = np.argmax([tr, M[0, 0], M[1, 1], M[2, 2]])
        if i == 0:
            s = 2 * np.sqrt(1 + tr)
            q[k] = [s / 4, (M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s, (M[1, 0] - M[0, 1]) / s]
        elif i == 1:
            s = 2 * np.sqrt(1 + M[0, 0] - M[1, 1] - M[2, 2])
            q[k] = [(M[2, 1] - M[1, 2]) / s, s / 4, (M[0, 1] + M[1, 0]) / s, (M[0, 2] + M[2, 0]) / s]
        elif i == 2:
            s = 2 * np.sqrt(1 - M[0, 0] + M[1, 1] - M[2, 2])
            q[k] = [(M[0, 2] - M[2, 0]) / s, (M[0, 1] + M[1, 0]) / s, s / 4, (M[1, 2] + M[2, 1]) / s]
        else:
            s = 2 * np.sqrt(1 - M[0, 0] - M[1, 1] + M[2, 2])
            q[k] = [(M[1, 0] - M[0, 1]) / s, (M[0, 2] + M[2, 0]) / s, (M[1, 2] + M[2, 1]) / s, s / 4]
    q *= np.where(q[:, :1] < 0, -1, 1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q[0] if single else q


def module3_target_angles(R):
    """Roll/pitch/yaw [deg] to type into Module 3's 'Target pose' boxes.
    That page uses three.js Euler order 'XYZ', i.e. R = Rx(roll) Ry(pitch) Rz(yaw)
    (NOT the URDF convention Rz Ry Rx). Example: tool-down diag(1,-1,-1) -> (180, 0, 0)."""
    R = np.asarray(R, float)
    pitch = np.arcsin(np.clip(R[0, 2], -1, 1))
    if abs(R[0, 2]) < 1 - 1e-9:
        roll, yaw = np.arctan2(-R[1, 2], R[2, 2]), np.arctan2(-R[0, 1], R[0, 0])
    else:                                    # gimbal lock: put everything in roll
        roll, yaw = np.arctan2(R[2, 1], R[1, 1]), 0.0
    return np.degrees([roll, pitch, yaw])


# --------------------------------------------------------------------- CSV writers
def _check_Q(Q):
    Q = np.atleast_2d(np.asarray(Q, float))
    if Q.shape[1] != 7 or not np.isfinite(Q).all():
        raise ValueError(f'expected finite (m, 7) array [x, q1..q6], got {Q.shape}')
    return Q


def write_joint_csv(path, Q, header=True):
    """Joint path / configuration list for modules 2, 4 and 5 (rows = samples)."""
    Q = _check_Q(Q)
    np.savetxt(path, Q, delimiter=',', fmt='%.9f',
               header=JOINT_HEADER if header else '', comments='')
    return path


def write_workspace_csv(path, T):
    """Prescribed end-effector poses (m, 4, 4) in the world frame -> module-5 workspace CSV."""
    T = np.asarray(T, float).reshape(-1, 4, 4)
    data = np.column_stack([T[:, :3, 3], rot_to_quat(T[:, :3, :3])])
    np.savetxt(path, data, delimiter=',', fmt='%.9f', header='x,y,z,qw,qx,qy,qz', comments='')
    return path


def write_ik_solutions_csv(path, T_W, x):
    """All admissible analytical IK solutions of one world pose with the base at x
    (module 3; also usable as the module-4 configuration list).
    The branch labels go to <path>.labels.txt in the same row order, because the
    simulator only accepts purely numeric 7-value rows."""
    q, st = ik_world(np.asarray(T_W, float), x)
    keep = np.flatnonzero(st[0] == OK)
    Q = np.column_stack([np.full(len(keep), float(x)), q[0, keep]])
    write_joint_csv(path, Q)
    labels = [BRANCH_LABELS[b] for b in keep]
    with open(str(path) + '.labels.txt', 'w') as f:
        f.write('# row  branch  (simulator row numbering starts at 1 after the header)\n')
        for i, lab in enumerate(labels, 1):
            f.write(f'{i:4d}  {lab}\n')
    return Q, labels


def check_joint_vs_workspace(Q, T, tol=1e-6):
    """Max position/rotation mismatch between FK(Q) and the prescribed poses T."""
    Tk = fk_dh(_check_Q(Q))
    T = np.asarray(T, float).reshape(-1, 4, 4)
    dp = np.linalg.norm(Tk[:, :3, 3] - T[:, :3, 3], axis=1).max()
    dR = np.linalg.norm(Tk[:, :3, :3] - T[:, :3, :3], axis=(1, 2)).max()
    return dp, dR


# --------------------------------------------------------------- module-2 WebSocket
def stream_joint_path(Q, dt=0.05, host='localhost', port=8765, loop=False):
    """Serve Q (m, 7) to the Module-2 page.

    The page is the WebSocket *client*: start this function first, then press
    "Connect" in the 'Python / WebSocket' panel. Each row is sent as
        {"type": "joint_state", "q": [x, q1..q6], "unit": "rad"}
    (the prismatic value x is in metres; the page applies it as given).
    Requires `pip install websockets`. Stop with Ctrl+C."""
    import websockets

    Q = _check_Q(Q)

    async def handler(ws, *_):
        print('simulator connected, streaming', len(Q), 'samples')
        while True:
            t0 = time.perf_counter()
            for k, q in enumerate(Q):
                await ws.send(json.dumps({'type': 'joint_state', 'q': q.tolist(), 'unit': 'rad'}))
                await asyncio.sleep(max(0.0, t0 + (k + 1) * dt - time.perf_counter()))
            if not loop:
                break
        print('done')

    async def main():
        async with websockets.serve(handler, host, port):
            print(f'waiting on ws://{host}:{port} - press "Connect" in Module 2')
            await asyncio.Future()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
