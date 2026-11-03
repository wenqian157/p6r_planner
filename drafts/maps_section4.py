"""DRAFT (Report section 4) - branch-resolved reachable-position maps over discrete base positions.

For a fixed flange orientation, sweep the flange position over a horizontal grid (world frame, height Z_MAP)
and record per IK branch one of four outcomes:
    0 feasible | 1 no real IK | 2 joint-limit rejection | 3 singular (sigma_min(J_red) < SIG_EPS)
Repeat for x = -1, 0, +1 m, overlay in the world frame, and relate the boundaries to the critical curves
of the IK formulas (= the factors of det J_red).

Run from the repo root:   python drafts/maps_section4.py
Outputs: drafts/figures/map_*.png, drafts/maps_section4_output.txt (stdout)
"""
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e_full import *                       # ik_6r, T06_from_world, LABELS, SIGNS, DH_ROWS, fk_rows_v, ...
from ik_ur12e_full import _dh
from umr_params import UMR_LENGTH, UMR_WIDTH
from fk_dh import T_BASE, T_TOOL

OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
FEAS, NOIK, JLIM, SING = 0, 1, 2, 3
CLASS = ["feasible", "no real IK", "joint limit", "singular"]
CMAP = ListedColormap(["#2e7d32", "#e0e0e0", "#f9a825", "#c62828"])

# ------------------------------------------------------------------ map settings (choices to discuss)
Z_MAP = 0.60                          # flange height of the map plane [m] (~ shoulder height 0.42 + 0.18)
SIG_EPS = 0.05                        # regularity threshold on sigma_min(J_red)
XS = [-1.0, 0.0, 1.0]                 # base positions of the proposal
H = 0.02                              # grid step [m]
GX = np.arange(-2.6, 2.6 + 1e-9, H)
GY = np.arange(-1.6, 1.6 + 1e-9, H)
TASK = (-1.5, 1.5, 0.5, 0.9)          # task region (x0, x1, y0, y1): a 3 m long shelf / table edge at y = 0.5..0.9
# NOTE frames: the end frame of the chain is the ROS-Industrial 'flange' frame (T_TOOL = rpy(0, -pi/2, -pi/2)
# after DH frame 6), whose x-axis points OUT of the tool (= DH z6).  'tool0' would have z out.
# So the approach direction is column 0 of R (flange x).
R_DOWN = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]])      # flange x = -Z world (picking), y = +Y, z = +X
R_SIDE = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.]])      # flange x = +Y world (horizontal, facing the task side), z = +Z
ORIENT = {"down": R_DOWN, "horizontal": R_SIDE}


# ------------------------------------------------------------------ vectorised Jacobian (arm, DH frame 0)
def jac_red_v(q):
    """6x6 geometric Jacobian [v; w] of the flange origin, expressed in DH frame 0, for q (N,6).
    Same columns as jacobian_p6r.jacobian_reduced up to a constant rotation (frame 0 vs world),
    so singular values are identical."""
    N = len(q)
    T = np.broadcast_to(np.eye(4), (N, 4, 4))
    z, o = [], []
    for i, (d, a, al) in enumerate(DH_ROWS):
        z.append(T[:, :3, 2]); o.append(T[:, :3, 3])
        T = T @ _dh(q[:, i], d, a, al)
    p = T[:, :3, 3]
    J = np.zeros((N, 6, 6))
    for k in range(6):
        J[:, :3, k] = np.cross(z[k], p - o[k])
        J[:, 3:, k] = z[k]
    return J


def slide_col_frame0():
    """Slide column [x_world; 0] expressed in DH frame 0 (T_BASE has yaw pi -> -x)."""
    R0 = (origin(MOUNT_XYZ, (0, 0, 0)) @ T_BASE)[:3, :3]
    return np.r_[R0.T @ np.array([1.0, 0, 0]), np.zeros(3)]


def sig_min(J):
    return np.linalg.svd(J, compute_uv=False)[..., -1]


# ------------------------------------------------------------------ one map
def branch_map(R, x, z=Z_MAP):
    """Classes (ny, nx, 8), sigma_min(J_red) and sigma_min(J) (ny, nx, 8), and the IK joint values."""
    XX, YY = np.meshgrid(GX, GY)
    n = XX.size
    T_W = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    T_W[:, :3, :3] = R
    T_W[:, 0, 3], T_W[:, 1, 3], T_W[:, 2, 3] = XX.ravel(), YY.ravel(), z
    T06 = _T06_batch(T_W, x)
    q, st = ik_6r(T06)
    cls = np.where(st == OK, FEAS, np.where(st == JOINT_LIMIT, JLIM, NOIK))
    sr = np.full(st.shape, np.nan); sf = np.full(st.shape, np.nan)
    m = st == OK
    J = jac_red_v(q[m])
    sr[m] = sig_min(J)
    Jf = np.concatenate([np.broadcast_to(slide_col_frame0()[:, None], (len(J), 6, 1)), J], 2)
    sf[m] = sig_min(Jf)
    cls[m & (sr < SIG_EPS)] = SING
    sh = XX.shape + (8,)
    return cls.reshape(sh), sr.reshape(sh), sf.reshape(sh), q.reshape(XX.shape + (8, 6)), T06


def _T06_batch(T_W, x):
    """T06_from_world for many targets at one x (same formula, batched)."""
    # T06_from_world(T, x) = inv(T_world_0) @ T @ inv(T_TOOL);  with T = I it returns inv(T_world_0) @ inv(T_TOOL)
    inv_Tw0 = T06_from_world(np.eye(4), x) @ T_TOOL
    return inv_Tw0 @ T_W @ np.linalg.inv(T_TOOL)


# ------------------------------------------------------------------ critical curves (from the IK formulas)
def critical_fields(T06, b):
    """Scalar fields whose zero sets are the boundaries / singular curves of branch b:
       shoulder  r/d4 - 1          (r = |O5_xy| in frame 0)   -> boundary of 'no real IK' and S+/S- merge
       elbow     1 - |c3|                                      -> boundary of 'no real IK' and E+/E- merge
       wrist     |sin q5|                                      -> W+/W- merge (inside the feasible region)"""
    sS, sW, sE = SIGNS[b]
    p05 = T06[:, :3, 3] - d6 * T06[:, :3, 2]
    r = np.hypot(p05[:, 0], p05[:, 1])
    t1, _ = theta1(T06, sS)
    t5, t6 = theta5_6(T06, t1, sW)
    A1i = np.linalg.inv(_dh(t1, d1, 0.0, pi / 2)); A5i = np.linalg.inv(_dh(t5, d5, 0.0, -pi / 2))
    A6i = np.linalg.inv(_dh(t6, d6, 0.0, 0.0))
    T14 = A1i @ T06 @ A6i @ A5i
    c3 = (T14[:, 0, 3] ** 2 + T14[:, 1, 3] ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
    shape = (len(GY), len(GX))
    return {"shoulder": (r / d4 - 1).reshape(shape),
            "elbow": (1 - np.abs(c3)).reshape(shape),
            "wrist": np.abs(np.sin(t5)).reshape(shape)}     # >= 0 by branch: draw the level 0.1 (touches 0)


# ------------------------------------------------------------------ main
if __name__ == "__main__":
    t0 = time.perf_counter()
    rng = np.random.default_rng(4)

    # (0) vectorised Jacobian vs her jacobian_p6r (singular values must agree)
    Qs = rng.uniform(-pi, pi, (200, 6))
    Jv = jac_red_v(Qs)
    Jf = np.concatenate([np.broadcast_to(slide_col_frame0()[:, None], (200, 6, 1)), Jv], 2)
    e_r = max(np.abs(np.linalg.svd(Jv[i], compute_uv=False) -
                     np.linalg.svd(jacobian_reduced(np.r_[0.3, Qs[i]]), compute_uv=False)).max() for i in range(200))
    e_f = max(np.abs(np.linalg.svd(Jf[i], compute_uv=False) -
                     np.linalg.svd(jacobian(np.r_[0.3, Qs[i]]), compute_uv=False)).max() for i in range(200))
    print(f"(0) singular values, vectorised vs scripts/jacobian_p6r.py over 200 random q:  "
          f"J_red {e_r:.1e},  J {e_f:.1e}")

    # (0b) det J_red factorises into the three families:  det = a2 a3 s3 s5 (a2 c2 + a3 c23 + d5 s234)  (x d4-free)
    s = np.sin; c = np.cos
    q2, q3, q4, q5 = Qs[:, 1], Qs[:, 2], Qs[:, 3], Qs[:, 4]
    f_sh = a2 * c(q2) + a3 * c(q2 + q3) + d5 * s(q2 + q3 + q4)
    det_formula = a2 * a3 * s(q3) * s(q5) * f_sh
    det_num = np.linalg.det(Jv)
    print(f"(0b) det J_red = a2 a3 s3 s5 (a2 c2 + a3 c23 + d5 s234):  max |num - formula| = "
          f"{np.abs(det_num - det_formula).max():.1e}   (|det| up to {np.abs(det_num).max():.2f})")
    # shoulder factor = signed horizontal distance of O5 from axis 1, measured inside the arm plane
    T = fk_rows_v(Qs); p05 = T[:, :3, 3] - d6 * T[:, :3, 2]
    r = np.hypot(p05[:, 0], p05[:, 1])
    print(f"     shoulder factor^2 + d4^2 = r^2 (|O5_xy|^2):  max error = "
          f"{np.abs(f_sh ** 2 + d4 ** 2 - r ** 2).max():.1e}   -> factor = 0  <=>  r = d4")

    summary = {}
    for oname, R in ORIENT.items():
        maps = {}
        for x in XS:
            cls, sr, sf, q, T06 = branch_map(R, x)
            maps[x] = (cls, sr, sf, q, T06)

        # (1) shifted copies: map(x) at world X == map(0) at world X - x
        k = int(round(1.0 / H))
        c0, c1 = maps[0.0][0], maps[1.0][0]
        same = np.mean(c1[:, k:] == c0[:, :-k])
        print(f"\n===== orientation: tool {oname}, flange height z = {Z_MAP} m, grid {len(GX)} x {len(GY)}, "
              f"step {H} m, sigma_min threshold {SIG_EPS}")
        print(f"(1) map(x=+1) equals map(x=0) shifted by +1 m on {same * 100:.3f} % of overlapping cells")

        # (2) per-branch area of each class (x = 0)
        cls0, sr0, sf0 = maps[0.0][:3]
        A = H * H
        print(f"(2) area per class at x = 0 [m^2]          " + "  ".join(f"{n:>11s}" for n in CLASS))
        for b in range(8):
            print(f"    {LABELS[b]:10s}                     " +
                  "  ".join(f"{np.sum(cls0[..., b] == j) * A:11.2f}" for j in range(4)))
        nfeas = (cls0 == FEAS).sum(2)
        print(f"    cells with >= 1 feasible branch: {np.sum(nfeas > 0) * A:.2f} m^2;   "
              f"count histogram (0..8 branches): {np.bincount(nfeas.ravel(), minlength=9)}")

        # (3) singular cells: does the slide rescue them? (sigma_min(J) vs sigma_min(J_red))
        sg = cls0 == SING
        print(f"(3) singular cells (all branches, x = 0): {sg.sum()};  of these sigma_min(J) >= {SIG_EPS} "
              f"(slide rescues): {np.mean(sf0[sg] >= SIG_EPS) * 100:.1f} %")

        # (3b) which family is each singular cell, and does the slide rescue it?
        #      family = factor of det J_red closest to zero:  elbow |s3|, wrist |s5|, shoulder |f_sh| / (|a2|+|a3|)
        q0 = maps[0.0][3][sg]                                   # (n, 6) joint vectors of singular cells
        f_sh = a2 * np.cos(q0[:, 1]) + a3 * np.cos(q0[:, 1] + q0[:, 2]) + d5 * np.sin(q0[:, 1:4].sum(1))
        fam = np.argmin(np.c_[np.abs(f_sh) / (abs(a2) + abs(a3)), np.abs(np.sin(q0[:, 2])), np.abs(np.sin(q0[:, 4]))], 1)
        resc = sf0[sg] >= SIG_EPS
        for i, nm in enumerate(["shoulder", "elbow", "wrist"]):
            if np.any(fam == i):
                print(f"      {nm:8s}: {np.sum(fam == i):5d} cells, slide rescues {np.mean(resc[fam == i]) * 100:5.1f} %")
            else:
                print(f"      {nm:8s}:     0 cells")

        # (4) task-region coverage per x and union
        XX, YY = np.meshgrid(GX, GY)
        inT = (XX >= TASK[0]) & (XX <= TASK[1]) & (YY >= TASK[2]) & (YY <= TASK[3])
        cov = {x: (maps[x][0] == FEAS).any(2) & inT for x in XS}
        union = np.logical_or.reduce([cov[x] for x in XS])
        print(f"(4) task region x in [{TASK[0]}, {TASK[1]}], y in [{TASK[2]}, {TASK[3]}]: covered by >= 1 feasible branch")
        for x in XS:
            print(f"      base at x = {x:+.0f} m : {cov[x].sum() / inT.sum() * 100:5.1f} %")
        print(f"      union of the three  : {union.sum() / inT.sum() * 100:5.1f} %   "
              f"(only one x works on {np.mean((sum(cov[x].astype(int) for x in XS) == 1)[inT]) * 100:.1f} %)")
        per_b = [np.logical_or.reduce([(maps[x][0][..., b] == FEAS) & inT for x in XS]).sum() / inT.sum() * 100
                 for b in range(8)]
        print("      per branch, union over x: " + ", ".join(f"{LABELS[b]} {per_b[b]:.0f}%" for b in range(8)))
        summary[oname] = maps

        # ---------------- figure A: 8 branch maps at x = 0, with critical curves
        fig, axs = plt.subplots(2, 4, figsize=(17, 7.6), sharex=True, sharey=True)
        T06 = maps[0.0][4]
        for b, ax in enumerate(axs.flat):
            ax.imshow(cls0[..., b], origin="lower", cmap=CMAP, vmin=-0.5, vmax=3.5,
                      extent=[GX[0] - H / 2, GX[-1] + H / 2, GY[0] - H / 2, GY[-1] + H / 2], interpolation="nearest")
            F = critical_fields(T06, b)
            ax.contour(GX, GY, F["shoulder"], [0], colors="k", linewidths=1.0, linestyles="--")
            ax.contour(GX, GY, F["elbow"], [0], colors="b", linewidths=1.0)
            ax.contour(GX, GY, np.where(cls0[..., b] == NOIK, np.nan, F["wrist"]), [0.1], colors="m", linewidths=1.0)
            ax.add_patch(Rectangle((-UMR_LENGTH / 2, -UMR_WIDTH / 2), UMR_LENGTH, UMR_WIDTH, fill=False, ec="k", lw=0.8))
            ax.set_title(LABELS[b], fontsize=10); ax.set_aspect("equal")
        for ax in axs[1]: ax.set_xlabel("world X [m]")
        for ax in axs[:, 0]: ax.set_ylabel("world Y [m]")
        handles = [plt.Rectangle((0, 0), 1, 1, color=CMAP(i)) for i in range(4)] + \
                  [plt.Line2D([], [], color="k", ls="--"), plt.Line2D([], [], color="b"), plt.Line2D([], [], color="m")]
        fig.legend(handles, CLASS + ["shoulder r = d4", "elbow |c3| = 1", "wrist |s5| = 0.1"], loc="lower center",
                   ncol=7, fontsize=9, frameon=False)
        fig.suptitle(f"Branch-resolved reachable positions, tool {oname}, flange at z = {Z_MAP} m, base at x = 0 "
                     f"(singular: sigma_min(J_red) < {SIG_EPS})", fontsize=12)
        fig.tight_layout(rect=(0, 0.04, 1, 1)); fig.savefig(OUT / f"map_{oname}_branches_x0.png", dpi=120); plt.close(fig)

        # ---------------- figure B: world overlay x = -1, 0, +1 (number of feasible branches) + union
        fig, axs = plt.subplots(2, 2, figsize=(13, 7.6), sharex=True, sharey=True)
        axs = axs.ravel()
        for ax, x in zip(axs[:3], XS):
            im = ax.imshow((maps[x][0] == FEAS).sum(2), origin="lower", cmap="Greens", vmin=0, vmax=8,
                           extent=[GX[0], GX[-1], GY[0], GY[-1]])
            ax.add_patch(Rectangle((x - UMR_LENGTH / 2, -UMR_WIDTH / 2), UMR_LENGTH, UMR_WIDTH, fill=False, ec="k"))
            ax.add_patch(Rectangle((TASK[0], TASK[2]), TASK[1] - TASK[0], TASK[3] - TASK[2], fill=False, ec="C3", lw=1.5))
            ax.set_title(f"base at x = {x:+.0f} m: number of feasible branches", fontsize=10)
        for ax in axs[2:]: ax.set_xlabel("world X [m]")
        for ax in axs[::2]: ax.set_ylabel("world Y [m]")
        fig.colorbar(im, ax=axs.tolist(), shrink=0.6, label="# feasible branches (of 8)")
        rgb = np.stack([cov_x.astype(float) for cov_x in
                        [(maps[x][0] == FEAS).any(2) for x in XS]], -1)       # R: x=-1, G: x=0, B: x=+1
        axs[3].imshow(rgb, origin="lower", extent=[GX[0], GX[-1], GY[0], GY[-1]])
        axs[3].add_patch(Rectangle((TASK[0], TASK[2]), TASK[1] - TASK[0], TASK[3] - TASK[2], fill=False, ec="w", lw=1.5))
        axs[3].set_title("overlay: red x=-1, green x=0, blue x=+1\n(mixtures = covered by several x)", fontsize=10)
        fig.suptitle(f"World-frame overlay, tool {oname}, z = {Z_MAP} m (red/white box: task region)")
        fig.savefig(OUT / f"map_{oname}_overlay.png", dpi=120, bbox_inches="tight"); plt.close(fig)

        # ---------------- figure C: sigma_min(J_red) vs sigma_min(J) for one branch, x = 0
        b = 2   # S+ W- E+  (the branch of the section 1-2 configuration)
        fig, axs = plt.subplots(1, 2, figsize=(13, 4.6), sharey=True)
        for ax, F, t in zip(axs, [sr0[..., b], sf0[..., b]], ["sigma_min(J_red), x fixed", "sigma_min(J), slide free"]):
            im = ax.imshow(F, origin="lower", cmap="viridis", vmin=0, vmax=0.5, extent=[GX[0], GX[-1], GY[0], GY[-1]])
            ax.contour(GX, GY, np.nan_to_num(F, nan=1), [SIG_EPS], colors="r", linewidths=0.8)
            ax.set_title(f"{LABELS[b]}, tool {oname}:\n{t}", fontsize=10); ax.set_xlabel("world X [m]"); ax.set_aspect("equal")
        axs[0].set_ylabel("world Y [m]"); fig.colorbar(im, ax=axs, shrink=0.85)
        fig.savefig(OUT / f"map_{oname}_sigma_{b}.png", dpi=120, bbox_inches="tight"); plt.close(fig)

    # (5) threshold sensitivity (tool down, x = 0)
    print("\n(5) sensitivity of the singular class to the threshold (tool down, x = 0, all branches)")
    cls_s, sr_s = summary["down"][0.0][0], summary["down"][0.0][1]
    reach = cls_s != NOIK
    for eps in (0.01, 0.02, 0.05, 0.1):
        print(f"      sigma_min < {eps:<5}: {np.mean(sr_s[reach] < eps) * 100:5.1f} % of the cells with real IK")
    print(f"\ndone in {time.perf_counter() - t0:.1f} s; figures in {OUT}")
