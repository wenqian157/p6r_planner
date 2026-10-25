"""Step 1: pull the six joints out of ur12e.urdf and print their origin (xyz, rpy) and axis.

Run from the repo root:   python scripts/step01_extract_joints.py
Also writes ur12e/ur12e_joints.yaml for later steps.
"""
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from p6r.urdf import load_chain

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "ur12e" / "ur12e.urdf"

chain = load_chain(URDF, root="world", tip="flange")
moving = [j for j in chain if j.moving]

print(f"{'#':2s} {'joint':20s} {'parent link':18s} {'child link':16s} "
      f"{'xyz [m]':30s} {'rpy [rad]':28s} axis")
for i, j in enumerate(moving, 1):
    xyz = "[" + ", ".join(f"{v:9.5f}" for v in j.xyz) + "]"
    rpy = "[" + ", ".join(f"{v:7.4f}" for v in j.rpy) + "]"
    ax = "[" + ", ".join(f"{v:.0f}" for v in j.axis) + "]"
    print(f"{i:<2d} {j.name:20s} {j.parent:18s} {j.child:16s} {xyz:30s} {rpy:28s} {ax}")

print("\nfixed transforms on the chain (not joints, but part of the geometry):")
for j in chain:
    if not j.moving:
        print(f"   {j.name:28s} {j.parent:18s} -> {j.child:18s} xyz={np.round(j.xyz, 5)}  rpy={np.round(j.rpy, 5)}")

# save for later steps
data = {
    "joints": [
        {"name": j.name, "parent": j.parent, "child": j.child,
         "xyz": [float(v) for v in j.xyz], "rpy": [float(v) for v in j.rpy],
         "axis": [float(v) for v in j.axis]}
        for j in moving
    ],
    "fixed": [
        {"name": j.name, "parent": j.parent, "child": j.child,
         "xyz": [float(v) for v in j.xyz], "rpy": [float(v) for v in j.rpy]}
        for j in chain if not j.moving
    ],
}
out = ROOT / "ur12e" / "ur12e_joints.yaml"
out.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
print("\nsaved", out.relative_to(ROOT))
