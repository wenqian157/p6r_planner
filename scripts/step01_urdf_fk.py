"""Step 1: read the UR12e URDF and compute the reference forward kinematics.

Prints the joint table, the moving-joint axes expressed in the base frame at q = 0
(these become the screw axes in step 3), and the flange pose at q = 0 and at a test q.

Run from the repo root:   python scripts/step01_urdf_fk.py
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from p6r.urdf import UrdfChain
from p6r.se3 import R_of, p_of

np.set_printoptions(precision=5, suppress=True)
URDF = Path(__file__).resolve().parents[1] / "ur12e" / "ur12e.urdf"

arm = UrdfChain(URDF, root="world", tip="flange")

print("=== joint chain world -> flange ===")
print(arm.table())
print(f"\nmoving joints ({arm.n}):", arm.names)
print("lower:", arm.lower)
print("upper:", arm.upper)

# joint axes in the world/base frame at the zero configuration
print("\n=== joint axes at q = 0, expressed in the world frame ===")
q0 = np.zeros(arm.n)
F = arm.frames(q0)
print(f"{'joint':20s} {'point on axis (world)':32s} direction (world)")
for j in arm.moving:
    Tc = F[j.child]                      # child frame = joint frame
    r = p_of(Tc)
    w = R_of(Tc) @ j.axis
    print(f"{j.name:20s} {np.array2string(r, precision=5):32s} {np.array2string(w, precision=3)}")

print("\n=== flange pose at q = 0 ===")
print(arm.fk(q0))

q_test = np.deg2rad([10, -60, 90, -120, -90, 30])
print("\n=== flange pose at q_test =", np.rad2deg(q_test), "deg ===")
print(arm.fk(q_test))

print("\n=== same q_test, tool0 frame ===")
tool = UrdfChain(URDF, root="world", tip="tool0")
print(tool.fk(q_test))
