"""
Kinematics of the P6R whole body: AgileX UMR (prismatic x along world X) + UR12e.

Conventions (follow ENG-654):
  * joint vector  Q = [x, q1, ..., q6]   (x in metres, q in rad)
  * standard D-H   A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
  * PoE screws     S = [omega; v]  (angular first, Lecture 02)
  * Jacobians      linear velocity first  [v; omega]  (Lecture 05)
  * end effector   = UR 'tool0' frame (origin at the flange centre, z out of the flange)
All functions accept a single configuration (7,) or a batch (N, 7).
"""
import numpy as np
import xml.etree.ElementTree as ET

# ----------------------------------------------------------------- parameters
D1, A2, A3 = 0.1807, -0.6127, -0.57155
D4, D5, D6 = 0.17415, 0.11985, 0.11655
H_MOUNT = 0.41                       # UMR top-plate height
Q_LIM = np.array([[-1.5, 1.5]] + [[-np.pi, np.pi]] * 6)   # TA: arm limits +-pi

# UR 6R standard D-H table (theta = q_i, no offsets): rows (d, a, alpha)
UR_DH = np.array([[D1, 0.0, np.pi / 2],
                  [0.0, A2, 0.0],
                  [0.0, A3, 0.0],
                  [D4, 0.0, np.pi / 2],
                  [D5, 0.0, -np.pi / 2],
                  [D6, 0.0, 0.0]])

# P6R whole-body D-H table, row i: (theta_offset, d_offset, a, alpha, type)
#   row 1 prismatic : theta=0,          d = x
#   row 2 revolute  : theta=q1 - pi/2,  d = H_MOUNT + D1
#   rows 3-7        : the UR rows 2-6
P6R_DH = [(0.0,        0.0,           0.0, -np.pi / 2, 'P'),
          (-np.pi / 2, H_MOUNT + D1,  0.0,  np.pi / 2, 'R'),
          (0.0,        0.0,           A2,   0.0,       'R'),
          (0.0,        0.0,           A3,   0.0,       'R'),
          (0.0,        D4,            0.0,  np.pi / 2, 'R'),
          (0.0,        D5,            0.0, -np.pi / 2, 'R'),
          (0.0,        D6,            0.0,  0.0,       'R')]
# fixed world -> D-H frame B : z_B along world X (prismatic axis), x_B along world Y
T_W_B = np.array([[0., 0., 1., 0.],
                  [1., 0., 0., 0.],
                  [0., 1., 0., 0.],
                  [0., 0., 0., 1.]])


# ------------------------------------------------------------ small helpers
def _batch(Q):
    Q = np.asarray(Q, float)
    return (Q[None], True) if Q.ndim == 1 else (Q, False)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx                      # URDF convention: fixed-axis XYZ


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def dh_A(theta, d, a, alpha):
    """Vectorised standard D-H transform; theta, d arrays of shape (N,)."""
    theta, d = np.broadcast_arrays(np.asarray(theta, float), np.asarray(d, float))
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    A = np.zeros(theta.shape + (4, 4))
    A[..., 0, 0], A[..., 0, 1], A[..., 0, 2], A[..., 0, 3] = ct, -st * ca, st * sa, a * ct
    A[..., 1, 0], A[..., 1, 1], A[..., 1, 2], A[..., 1, 3] = st, ct * ca, -ct * sa, a * st
    A[..., 2, 1], A[..., 2, 2], A[..., 2, 3] = sa, ca, d
    A[..., 3, 3] = 1.0
    return A


def wrap(q):
    """Wrap angles to (-pi, pi]."""
    return np.pi - np.mod(np.pi - q, 2 * np.pi)


# ===================================================================== 1. URDF
class URDFChain:
    """Minimal URDF reader: serial chain root -> tip, FK of every link."""

    def __init__(self, path, root='world', tip='tool0'):
        joints = {}
        for j in ET.parse(path).getroot().findall('joint'):
            o = j.find('origin')
            xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            ax = j.find('axis')
            joints[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                T=np.block([[rpy_to_R(*rpy), xyz[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]),
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None)
        chain, link = [], tip
        while link != root:
            chain.append(joints[link]); link = joints[link]['parent']
        self.chain = chain[::-1]
        self.active = [j for j in self.chain if j['type'] != 'fixed']

    def fk(self, Q, frames=False):
        Q, single = _batch(Q)
        T = np.tile(np.eye(4), (len(Q), 1, 1))
        out, k = {}, 0
        for j in self.chain:
            T = T @ j['T']
            if j['type'] != 'fixed':
                q, a = Q[:, k], j['axis']; k += 1
                M = np.tile(np.eye(4), (len(Q), 1, 1))
                if j['type'] == 'prismatic':
                    M[:, :3, 3] = q[:, None] * a
                else:                                   # Rodrigues, vectorised
                    K = skew(a)
                    M[:, :3, :3] = (np.eye(3) + np.sin(q)[:, None, None] * K
                                    + (1 - np.cos(q))[:, None, None] * (K @ K))
                T = T @ M
            out[j['name']] = T.copy()
        if frames:
            return out
        return T[0] if single else T


# ======================================================================= 2. DH
def dh_frames(Q):
    """World-frame D-H frames F_0 .. F_7 for the P6R chain.  Returns (N, 8, 4, 4).
    F_0 = T_W_B (fixed);  F_i = F_{i-1} A_i;  joint i+1 acts about z of F_i."""
    Q, single = _batch(Q)
    N = len(Q)
    F = np.empty((N, 8, 4, 4))
    F[:, 0] = T_W_B
    for i, (th0, d0, a, al, typ) in enumerate(P6R_DH):
        theta = np.full(N, th0) if typ == 'P' else Q[:, i] + th0
        d = Q[:, i] + d0 if typ == 'P' else np.full(N, d0)
        F[:, i + 1] = F[:, i] @ dh_A(theta, d, a, al)
    return F[0] if single else F


def fk_dh(Q):
    F = dh_frames(Q)
    return F[..., -1, :, :]


def fk_ur(q):
    """Fixed-base UR12e FK in the UR 'base' (= D-H frame 0) frame.  q: (6,) or (N,6)."""
    q = np.asarray(q, float)
    single = q.ndim == 1
    q = np.atleast_2d(q)
    T = np.tile(np.eye(4), (len(q), 1, 1))
    for i in range(6):
        T = T @ dh_A(q[:, i], UR_DH[i, 0], UR_DH[i, 1], UR_DH[i, 2])
    return T[0] if single else T


def T_world_urbase(x):
    """World pose of the UR 'base' frame when the platform is at x."""
    x = np.atleast_1d(np.asarray(x, float))
    T = np.tile(np.eye(4), (len(x), 1, 1))
    T[:, 0, 3] = x
    T[:, 2, 3] = H_MOUNT
    return T


# ====================================================================== 3. PoE
def home_screws():
    """Space-frame screws S_i = [omega; v] and home pose M at Q = 0 (from D-H frames)."""
    F = dh_frames(np.zeros(7))
    S = np.zeros((6, 7))
    for i, (_, _, _, _, typ) in enumerate(P6R_DH):
        z, p = F[i, :3, 2], F[i, :3, 3]
        S[:, i] = np.r_[np.zeros(3), z] if typ == 'P' else np.r_[z, -np.cross(z, p)]
    return S, F[-1]


def exp_se3(S, theta):
    """Vectorised matrix exponential of a unit screw S=[w;v] times theta (N,)."""
    w, v = S[:3], S[3:]
    th = np.asarray(theta, float)
    E = np.tile(np.eye(4), (len(th), 1, 1))
    if np.allclose(w, 0):
        E[:, :3, 3] = th[:, None] * v
        return E
    K = skew(w)
    s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
    E[:, :3, :3] = np.eye(3) + s * K + (1 - c) * (K @ K)
    G = th[:, None, None] * np.eye(3) + (1 - c) * K + (th[:, None, None] - s) * (K @ K)
    E[:, :3, 3] = (G @ v)
    return E


S_HOME, M_HOME = home_screws()


def fk_poe(Q):
    Q, single = _batch(Q)
    T = np.tile(np.eye(4), (len(Q), 1, 1))
    for i in range(7):
        T = T @ exp_se3(S_HOME[:, i], Q[:, i])
    T = T @ M_HOME
    return T[0] if single else T


# ================================================================ 4. Jacobians
def jacobian_full(Q):
    """6x7 geometric Jacobian at the tool0 origin, world frame, [v; omega]."""
    F = dh_frames(Q)
    single = F.ndim == 3
    F = F[None] if single else F
    pt = F[:, -1, :3, 3]
    J = np.zeros((len(F), 6, 7))
    for i, (*_, typ) in enumerate(P6R_DH):
        z, p = F[:, i, :3, 2], F[:, i, :3, 3]
        if typ == 'P':
            J[:, :3, i] = z
        else:
            J[:, :3, i] = np.cross(z, pt - p)
            J[:, 3:, i] = z
    return J[0] if single else J


def jacobian_reduced(Q):
    """6x6 arm Jacobian for frozen x (columns of q1..q6)."""
    return jacobian_full(Q)[..., 1:]


def jacobian_space_poe(Q):
    """Space Jacobian from the PoE screws, [omega; v] (Lynch & Park), for cross-checking."""
    Q = np.asarray(Q, float)
    Js = np.zeros((6, 7))
    T = np.eye(4)
    for i in range(7):
        R, p = T[:3, :3], T[:3, 3]
        Ad = np.block([[R, np.zeros((3, 3))], [skew(p) @ R, R]])
        Js[:, i] = Ad @ S_HOME[:, i]
        T = T @ exp_se3(S_HOME[:, i], Q[i:i + 1])[0]
    return Js


def jacobian_3J5(Q):
    """Course 'wrist/subchain' Jacobian of the arm: velocity of O5 (origin of UR D-H
    frame 5, which lies on axes 5 and 6), both blocks expressed in UR D-H frame 3.
    6x6, columns q1..q6, [v; omega]."""
    F = dh_frames(Q)
    o5 = F[6, :3, 3]                          # UR frame 5 = P6R frame index 6
    R3 = F[4, :3, :3]                         # UR frame 3 = P6R frame index 4
    J = np.zeros((6, 6))
    for k in range(6):
        z, p = F[k + 1, :3, 2], F[k + 1, :3, 3]
        J[:3, k] = R3.T @ np.cross(z, o5 - p)
        J[3:, k] = R3.T @ z
    return J


def sigma_min(J):
    return np.linalg.svd(J, compute_uv=False)[..., -1]
