"""Label persistence of scripts/ik_ur12e.py along smooth joint-space paths (Report section 3.4).
100 random straight joint paths, 300 steps each: FK -> IK at every step; for each branch label, is the
nearest root at the next step (over all branches, wrapped angles) the same label?  Every switch is
classified by the closest singular family.  Run from the repo root:  python drafts/ik_persistence_check.py"""
import os, sys
from pathlib import Path
import numpy as np
ROOT = Path(os.environ.get("P6R_ROOT", Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(ROOT / "scripts"))
from ik_ur12e import ik, fk_rows, a2, a3, d5

pi = np.pi
wrap = lambda a: (a + pi) % (2 * pi) - pi
rng = np.random.default_rng(2)
tot = viol = 0
fam = []
for _ in range(100):
    qa, qb = rng.uniform(-2.5, 2.5, 6), rng.uniform(-2.5, 2.5, 6)
    prev = None
    for t in np.linspace(0, 1, 300):
        sols = {s["label"]: s["q"] for s in ik(fk_rows(qa + t * (qb - qa)))}
        if prev:
            for lab, q0 in prev.items():
                if lab not in sols:
                    continue
                near = min(sols, key=lambda L: np.linalg.norm(wrap(sols[L] - q0)))
                tot += 1
                if near != lab:
                    viol += 1
                    q = q0
                    rho = a2 * np.cos(q[1]) + a3 * np.cos(q[1] + q[2]) + d5 * np.sin(q[1:4].sum())
                    fam.append(np.argmin([abs(rho) / (abs(a2) + abs(a3)), abs(np.sin(q[2])), abs(np.sin(q[4]))]))
        prev = sols
fam = np.array(fam)
print(f"label persistence: nearest root at the next step has the same label in {100 * (1 - viol / tot):.2f} % of {tot} steps")
print(f"the {viol} switches, by closest singular family (smallest normalised det factor):")
for k, n in enumerate(["shoulder (rho = 0)", "elbow (sin q3 = 0)", "wrist (sin q5 = 0)"]):
    print(f"  {n:20s}: {np.mean(fam == k) * 100:5.1f} %")
