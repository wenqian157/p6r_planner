"""Forward kinematics of the UR12e straight from the URDF numbers.

For each joint, in order from the base to the flange:
    T = T @ Origin(xyz, rpy) @ Rot(axis, q_i)
Origin is the joint's <origin> (pose of the child frame in the parent frame),
Rot(axis, q_i) is the joint rotation about the joint's <axis>, given in the child frame
(for the UR12e every axis is 0 0 1, i.e. the child's z).
Fixed transforms (base yaw, flange) have no axis and contribute only their Origin.

Run from the repo root:
    python scripts/fk_urdf.py                      -> q = 0
    python scripts/fk_urdf.py 10 -60 90 -120 -90 30  -> q in degrees
"""
import sys
from pathlib import Path
import numpy as np

np.set_printoptions(precision=5, suppress=True)
pi = np.pi

# ---------------------------------------------------------------- the URDF numbers
# (name, xyz, rpy, axis)  in chain order, each relative to the previous frame.
# axis is the joint's rotation axis in its own (child) frame; None = fixed transform.
CHAIN = [
    ("base_link -> base_link_inertia", [0, 0, 0],              [0, 0, pi],        None),
    ("1 shoulder_pan",                 [0, 0, 0.1807],         [0, 0, 0],         [0, 0, 1]),
    ("2 shoulder_lift",                [0, 0, 0],              [pi/2, 0, 0],      [0, 0, 1]),
    ("3 elbow",                        [-0.6127, 0, 0],        [0, 0, 0],         [0, 0, 1]),
    ("4 wrist_1",                      [-0.57155, 0, 0.17415], [0, 0, 0],         [0, 0, 1]),
    ("5 wrist_2",                      [0, -0.11985, 0],       [pi/2, 0, 0],      [0, 0, 1]),
    ("6 wrist_3",                      [0, 0.11655, 0],        [pi/2, pi, pi],    [0, 0, 1]),
    ("wrist_3 -> flange",              [0, 0, 0],              [0, -pi/2, -pi/2], None),
]


# ---------------------------------------------------------------- small helpers
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def origin(xyz, rpy):
    """URDF <origin>: R = Rz(yaw) Ry(pitch) Rx(roll), then translation xyz."""
    roll, pitch, yaw = rpy
    T = np.eye(4)
    T[:3, :3] = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    T[:3, 3] = xyz
    return T

def rot_axis(axis, q):
    """Rotation by q about a unit axis (Rodrigues): R = I + sin(q) K + (1 - cos(q)) K^2."""
    w = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    return T


# ---------------------------------------------------------------- forward kinematics
def fk(q, verbose=False):
    """T_world_flange for the six joint angles q [rad]."""
    T, k = np.eye(4), 0
    for name, xyz, rpy, axis in CHAIN:
        T = T @ origin(xyz, rpy)
        if axis is not None:
            T = T @ rot_axis(axis, q[k])
            k += 1
        if verbose:
            print(f"after {name:32s} position = {T[:3, 3]}")
    return T


if __name__ == "__main__":
    q = np.deg2rad([float(v) for v in sys.argv[1:]]) if len(sys.argv) == 7 else np.zeros(6)
    print("q [deg] =", np.rad2deg(q))
    T = fk(q, verbose=True)
    print("\nT_world_flange =")
    print(T)
    print("\nflange position [m] =", T[:3, 3])
