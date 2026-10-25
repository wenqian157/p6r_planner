"""Step 2: Jacobians, 3J5 factorisation, and which arm singularities survive the base."""
import numpy as np, sympy as sp
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import brentq
from p6r.kinematics import *

np.set_printoptions(precision=4, suppress=True)
rng = np.random.default_rng(1)

# ------------------------------------------------ (a) numerical consistency checks
Q = np.r_[0.3, rng.uniform(-np.pi, np.pi, 6)]
J = jacobian_full(Q)
eps, Jfd = 1e-6, np.zeros((3, 7))
for k in range(7):
    dQ = np.zeros(7); dQ[k] = eps
    Jfd[:, k] = (fk_dh(Q + dQ)[:3, 3] - fk_dh(Q - dQ)[:3, 3]) / (2 * eps)
print('Linear rows vs central differences: max err %.1e' % abs(J[:3] - Jfd).max())
Js = jacobian_space_poe(Q)                        # [omega; v_origin]
pt = fk_dh(Q)[:3, 3]
J_from_poe = np.vstack([Js[3:] + np.cross(Js[:3].T, pt).T, Js[:3]])
print('Geometric J vs PoE space Jacobian (shifted to tool, reordered): max err %.1e'
      % abs(J - J_from_poe).max())
print('Prismatic column:', J[:, 0])
print('\nFull 6x7 Jacobian at a random Q:\n', J)

# ------------------------------------------------ (b) symbolic 3J5 and its determinant
q = sp.symbols('q1:7', real=True)
d1, a2, a3, d4, d5, d6 = sp.symbols('d1 a2 a3 d4 d5 d6', real=True)
dh = [(q[0], d1, 0, sp.pi / 2), (q[1], 0, a2, 0), (q[2], 0, a3, 0),
      (q[3], d4, 0, sp.pi / 2), (q[4], d5, 0, -sp.pi / 2), (q[5], d6, 0, 0)]


def A(th, d, a, al):
    return sp.Matrix([[sp.cos(th), -sp.sin(th) * sp.cos(al), sp.sin(th) * sp.sin(al), a * sp.cos(th)],
                      [sp.sin(th), sp.cos(th) * sp.cos(al), -sp.cos(th) * sp.sin(al), a * sp.sin(th)],
                      [0, sp.sin(al), sp.cos(al), d], [0, 0, 0, 1]])


T = [sp.eye(4)]
for row in dh:
    T.append(sp.simplify(T[-1] * A(*row)))
o5, R3 = T[5][:3, 3], T[3][:3, :3]
cols = []
for k in range(6):
    z, p = T[k][:3, 2], T[k][:3, 3]
    cols.append(sp.Matrix.vstack(R3.T * z.cross(o5 - p), R3.T * z))
J35 = sp.simplify(sp.Matrix.hstack(*cols))
print('\nSymbolic 3J5 (UR, velocity of O5 in D-H frame 3):')
sp.pprint(J35)
detJ = sp.factor(sp.trigsimp(J35.det()))
print('\ndet(3J5) =', detJ)
det_f = sp.lambdify((q, d1, a2, a3, d4, d5, d6), detJ, 'numpy')
qq = rng.uniform(-np.pi, np.pi, 6)
print('check det formula vs numeric det of reduced J: %.3e vs %.3e'
      % (det_f(qq, D1, A2, A3, D4, D5, D6), np.linalg.det(jacobian_reduced(np.r_[0, qq]))))
print('check numeric 3J5 equals symbolic: %.1e' % abs(
    np.array(sp.lambdify((q, d1, a2, a3, d4, d5, d6), J35)(qq, D1, A2, A3, D4, D5, D6), float)
    - jacobian_3J5(np.r_[0, qq])).max())

# ------------------------------------------------ (c) singular configurations
shoulder_term = lambda q2, q3, q4: A2 * np.cos(q2) + A3 * np.cos(q2 + q3) + D5 * np.sin(q2 + q3 + q4)


def sing_config(kind, q1):
    """A configuration exactly on one singular family (other joints generic)."""
    base = np.array([q1, -1.0, 1.3, -0.9, 0.8, 0.4])
    if kind == 'elbow':
        base[2] = 0.0                                   # s3 = 0 : arm stretched
    elif kind == 'wrist':
        base[4] = 0.0                                   # s5 = 0 : axes 4 and 6 parallel
    elif kind == 'shoulder':                            # solve shoulder term for q2
        f = lambda t: shoulder_term(t, base[2], base[3])
        g = np.linspace(-np.pi, np.pi, 721); v = f(g)
        i = np.flatnonzero(np.sign(v[:-1]) != np.sign(v[1:]))[0]
        base[1] = brentq(f, g[i], g[i + 1])
    return np.r_[0.0, base]


def lost_direction(Q):
    """Left-null vector n of the reduced Jacobian (n^T J_red = 0) = reciprocal wrench."""
    U, s, Vt = np.linalg.svd(jacobian_reduced(Q))
    return U[:, -1], s[-1]


q1s = np.linspace(-np.pi, np.pi, 361)
res = {}
print('\n kind     | q1=0: sig_min(J_red) sig_min(J_full) |n_x|   lost wrench n = [force; moment]')
for kind in ['shoulder', 'elbow', 'wrist']:
    s_red, s_full, nx = [], [], []
    for q1 in q1s:
        Qs = sing_config(kind, q1)
        n, sr = lost_direction(Qs)
        s_red.append(sr); s_full.append(sigma_min(jacobian_full(Qs))); nx.append(abs(n[0]))
    res[kind] = (np.array(s_red), np.array(s_full), np.array(nx))
    Qs = sing_config(kind, 0.0); n, sr = lost_direction(Qs)
    print(f' {kind:8s} | {sr:.1e}  {sigma_min(jacobian_full(Qs)):.3f}  {abs(n[0]):.3f}   {n}')

fig, ax = plt.subplots(1, 3, figsize=(13, 3.6), sharey=True)
for a, kind in zip(ax, res):
    s_red, s_full, nx = res[kind]
    a.plot(np.degrees(q1s), s_red, label=r'$\sigma_{min}(J_{red})$ (6x6, x frozen)')
    a.plot(np.degrees(q1s), s_full, label=r'$\sigma_{min}(J_{full})$ (6x7)')
    a.plot(np.degrees(q1s), nx, '--', label=r'$|n_x|$: X-force in lost wrench')
    a.set_title(f'{kind} singularity'); a.set_xlabel(r'$q_1$ [deg]'); a.grid(alpha=.3)
ax[0].legend(fontsize=8)
plt.tight_layout(); plt.savefig('figures/step2_singularity_recovery.png', dpi=150)
print('saved figures/step2_singularity_recovery.png')
