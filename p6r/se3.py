"""Rigid-body helpers (plain numpy). Angles in radians, vectors length 3, transforms 4x4.

Same conventions as the course kinematics_tools.py:
  URDF rpy  ->  R = Rz(yaw) Ry(pitch) Rx(roll)
  twists are angular-first [w; v]
"""
import numpy as np


# ---------------------------------------------------------------- rotations
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(roll, pitch, yaw):
    """URDF convention."""
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def rodrigues(w, theta):
    """exp([w] theta) for unit w."""
    K = skew(w)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


# --------------------------------------------------------------- transforms
def T_from(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def R_of(T):
    return T[:3, :3]


def p_of(T):
    return T[:3, 3]


def T_inv(T):
    R, p = R_of(T), p_of(T)
    return T_from(R.T, -R.T @ p)


def urdf_origin(xyz, rpy):
    """<origin xyz rpy/> as a 4x4 transform."""
    return T_from(rpy_to_R(*rpy), xyz)


def T_rot_axis(axis, q):
    """Rotation by q about a unit axis through the origin (revolute joint motion)."""
    return T_from(R=rodrigues(np.asarray(axis, float), q))


def T_trans_axis(axis, q):
    """Translation by q along a unit axis (prismatic joint motion)."""
    return T_from(p=q * np.asarray(axis, float))


# ------------------------------------------------------------ comparisons
def pose_error(T_a, T_b):
    """(position error [m], orientation error [rad]) between two transforms."""
    e_p = np.linalg.norm(p_of(T_a) - p_of(T_b))
    c = (np.trace(R_of(T_b).T @ R_of(T_a)) - 1) / 2
    e_R = np.arccos(np.clip(c, -1.0, 1.0))
    return e_p, e_R


def wrap_pi(a):
    """Wrap angle(s) to (-pi, pi]."""
    a = np.asarray(a, float)
    return -((-a + np.pi) % (2 * np.pi) - np.pi)
