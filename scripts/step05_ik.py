"""Step 5: validate the analytical IK of the UR12e and show its branches (lecture-6 slide 18/19 style).

  1. random configurations: FK -> IK recovers the configuration, every branch maps back to the pose
  2. one pose, all branches: joint table with labels and the sign triple (rho, sin q3, sin q5)
  3. branch label == sign triple read off the joint vector (non-cuspidal check, 2000 random q)
  4. the three singular family poses: branches merge in pairs
  5. the same world pose seen from several base positions x: which branches exist

Run from the repo root:   python scripts/step05_ik.py
Output printed and written to results/step05_ik.md
"""
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ik_ur12e import ik, ik_world, fk_rows, branch_of, T06_from_world, wrap, a2, a3, d5
from fk_urdf_p6r import fk as fk_p6r
from jacobian_p6r import jacobian_reduced


class Tee:
    def __init__(self): self.buf, self.out = [], sys.stdout
    def write(self, t): self.buf.append(t); self.out.write(t)
    def flush(self): self.out.flush()


sys.stdout = tee = Tee()
pi = np.pi
np.set_printoptions(precision=4, suppress=True, linewidth=150)
deg = np.rad2deg
rho = lambda q: a2 * np.cos(q[1]) + a3 * np.cos(q[1] + q[2]) + d5 * np.sin(q[1] + q[2] + q[3])
sgn = lambda v: "+" if v >= 0 else "-"


def table(sols, T_ref=None):
    print(f"  {'branch':10s} {'q1':>8s} {'q2':>8s} {'q3':>8s} {'q4':>8s} {'q5':>8s} {'q6':>8s}   {'rho':>7s} {'sin q3':>7s} {'sin q5':>7s}  {'det J_red':>10s}  {'|FK-T|':>8s}")
    for s in sols:
        q = s["q"]
        e = np.abs(fk_rows(q) - T_ref).max() if T_ref is not None else float("nan")
        print(f"  {s['label']:10s} " + " ".join(f"{v:8.2f}" for v in deg(q))
              + f"   {rho(q):7.3f} {np.sin(q[2]):7.3f} {np.sin(q[4]):7.3f}  {np.linalg.det(jacobian_reduced(np.r_[0, q])):10.3e}  {e:8.1e}"
              + ("   (wrist singular, q6 set to 0)" if s["wrist_singular"] else ""))


# ---------------------------------------------------------------- 1. random configurations
rng = np.random.default_rng(0)
worst_match, worst_fk, counts, mismatch, n_sol = 0.0, 0.0, {}, 0, 0
for _ in range(2000):
    q = rng.uniform(-pi, pi, 6)
    T = fk_rows(q)
    sols = ik(T)
    counts[len(sols)] = counts.get(len(sols), 0) + 1
    worst_match = max(worst_match, min(np.abs(wrap(s["q"] - q)).max() for s in sols))
    worst_fk = max(worst_fk, max(np.abs(fk_rows(s["q"]) - T).max() for s in sols))
    for s in sols:
        n_sol += 1
        mismatch += branch_of(s["q"]) != s["label"]
print("=== 1. validation on 2000 random configurations q (joint limits +/- pi) ===")
print(f"  max |q_true - nearest branch|  = {worst_match:.1e} rad   (the input configuration is always recovered)")
print(f"  max |FK(branch) - T06|         = {worst_fk:.1e}       (every returned branch reaches the pose)")
print(f"  number of branches per pose    = {dict(sorted(counts.items()))}")
print("  (fewer than 8: the other shoulder or wrist choice puts O4 out of the planar 2R reach, |cos q3| > 1)")

# ---------------------------------------------------------------- 2. one pose, all branches
q0 = np.deg2rad([10, -60, 90, -120, -90, 30])
T0 = fk_rows(q0)
print(f"\n=== 2. all branches of one pose: T06 = FK(q0), q0 = {deg(q0)} deg ===")
table(ik(T0), T0)
print("  reading: the three sign columns are the branch label; every branch has a different sign triple,")
print("           so each one lives in its own aspect (region cut out by the three singular surfaces).")

# ---------------------------------------------------------------- 3. label vs sign triple
print(f"\n=== 3. branch label vs sign triple read from the joint vector ===")
print(f"  {n_sol} solutions over the 2000 random poses, label mismatches: {mismatch}")
print("  -> the label is a function of q alone: (sign rho, sign q5, sign q3).  Consistent with the UR being")
print("     non-cuspidal: one solution per aspect, and a change of branch needs a crossing of det J_red = 0.")

# ---------------------------------------------------------------- 4. singular family poses
print("\n=== 4. the three singular families: branches merge in pairs ===")
for name, qd in [("elbow    sin q3 = 0", [0, -45, 0, -45, -90, 0]),
                 ("wrist    sin q5 = 0", [10, -60, 90, -120, 0, 30]),
                 ("shoulder rho = 0   ", [0, 55, 90, 111.9529, -90, 0])]:
    qs = np.deg2rad(qd)
    Ts = fk_rows(qs)
    sols = ik(Ts)
    print(f"\n  {name}:  q = {qd} deg,  det J_red = {np.linalg.det(jacobian_reduced(np.r_[0, qs])):.1e}")
    table(sols, Ts)
    qs_all = np.array([s["q"] for s in sols])
    dup = sum(np.abs(wrap(qs_all[i] - qs_all[j])).max() < 1e-6 for i in range(len(sols)) for j in range(i + 1, len(sols)))
    print(f"  identical joint vectors among the branches: {dup} pair(s)  -> two labels, one configuration: the branch boundary")

# ---------------------------------------------------------------- 5. one world pose, several x
q7 = np.r_[0.5, q0]
T_world = fk_p6r(q7)
print(f"\n=== 5. one world pose (from x = 0.5 m, q0 above), IK of the arm for several base positions x ===")
print(f"  {'x [m]':>6s} {'branches':>8s}   labels")
for x in np.arange(-0.4, 1.45, 0.1):
    sols = ik_world(T_world, x)
    print(f"  {x:6.2f} {len(sols):8d}   {' '.join(s['label'].replace(' ', '') for s in sols)}")
print("  reading: the base position selects which branches exist.  This row of counts is one column of the")
print("           (s, x) plane: a fixed path point s, x swept.")

# ---------------------------------------------------------------- write results/step05_ik.md
sys.stdout = tee.out
out = HERE.parent / "results" / "step05_ik.md"
out.parent.mkdir(exist_ok=True)
header = "# Step 5: analytical IK of the UR12e, validation and branches" + chr(10) * 2 + "Script: `scripts/step05_ik.py`, solver: `scripts/ik_ur12e.py`" + chr(10) * 2
fence = "```"
out.write_text(header + fence + "text" + chr(10) + "".join(tee.buf).rstrip() + chr(10) + fence + chr(10), encoding="utf-8")
print(f"written to {out.relative_to(HERE.parent)}")
