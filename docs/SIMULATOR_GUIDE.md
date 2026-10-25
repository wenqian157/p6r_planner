# Using the ENG-654 course tools with our P6R (UMR + UR12e)

The TA's web simulators and playgrounds are built around the course robots (iiwa 7, GoFa, …),
but Modules 2–5 accept **any URDF + meshes + CSV files**. This guide explains how to drive them
with our own model and results, so every figure in the report can also be shown live in the
course simulators.

Course pages: <https://epfl-lasa.github.io/eng-654/> · Simulations: `…/modules/module1/` to `module5/`
· Playgrounds: `…/playground/building_blocks.html`, `…/playground/dh_parameters.html`

---

## 1. One-time setup (≈ 2 min)

From the repository root:

```bash
pip install trimesh pycollada websockets      # trimesh/pycollada only needed to rebuild meshes
python sim/build_sim_assets.py                # -> sim/meshes/*.stl, sim/umr_ur12e_sim.urdf
python sim/verify_sim_assets.py               # sim URDF == project URDF == D-H FK
python sim/export_examples.py                 # -> sim/data/*.csv  (ready-to-load examples)
```

(`sim/meshes` and `sim/umr_ur12e_sim.urdf` are committed, so the first command is only needed
if the source URDF or meshes change.)

What gets loaded into the simulators:

| File | Use |
|---|---|
| `sim/umr_ur12e_sim.urdf` | the robot, for every module (same joints as `umr_urdf/umr_ur12e.urdf`, plus visuals) |
| `sim/meshes/*.stl` (8 files) | "Visual mesh files": **select all 8 at once** (Ctrl/Shift-click) |
| `sim/data/*.csv` | example inputs (see §3) |

**Joint order everywhere = `Q = [x, q1, q2, q3, q4, q5, q6]`** (x in m, angles in rad). The
simulators read the movable joints in URDF order, which is exactly the `p6r` convention, so
arrays from our code can be written out without reordering. Module 5 (and Module 4) require
exactly 7 movable joints, which our P6R has.

---

## 2. Which tool supports which project step

| Project step (proposal) | Course tool | What to show |
|---|---|---|
| Kinematic model, URDF | Module 1 (Robot visualization) | load the URDF, display joint frames / axes, check the arm sits on the UMR at z = 0.41 m |
| DH / PoE / FK agreement | Module 2 sliders + Building-blocks playground | move each joint, compare EE x,y,z readout with `fk_dh`; rebuild the D-H chain from blocks |
| Analytical IK, all branches (expected output 5) | **Module 3** | 8 ghost robots for one pose, position/orientation error per solution = 0 |
| Redundancy of the base (x as self-motion) | **Module 4** | choose `base_x_joint` as redundancy joint, drag x, arm compensates, obstacle clearance |
| Paths, branch continuation, x(s) plans (outputs 7–9) | **Module 5** (+ Module 2 live stream) | prescribed path (white) vs. FK trace of our joint path (red) |

---

## 3. Module by module

### Module 1 – Robot visualization
1. URDF → `sim/umr_ur12e_sim.urdf`; visual meshes → all 8 files in `sim/meshes/`.
2. Turn on *joint frames* and *revolute-joint axes*. The prismatic axis of `base_x_joint` is along world X.
3. Use *Joint preview → custom pose* to compare poses with `p6r.kinematics.dh_frames`.

Do **not** drag meshes or joints here unless you mean to edit the URDF: Module 1 writes changes
into `<origin>` tags (that is Exercise 01's purpose).

### Module 2 – Joint commands & FK
- **Sliders**: one per movable joint (first slider = base x in metres). End-effector link: `tool0`.
  The x/y/z readout is the world position of `tool0`, which should equal `fk_dh(Q)[:3, 3]`.
- **CSV playback**: load any joint CSV from `sim/data/` (header row is skipped automatically;
  rows must have exactly 7 numbers). Set *Sample period* and press Play. The red line is the EE trace.
- **Live Python stream** (useful while debugging the planner):
  ```bash
  python sim/stream_to_module2.py                      # default: sim/data/m5_joint_moving_base.csv
  python sim/stream_to_module2.py my_path.csv --dt 0.02 --loop
  ```
  then press **Connect** in the *Python / WebSocket* panel. The page is the WebSocket *client*;
  our script is the server on `ws://localhost:8765`. From your own code:
  `from p6r.sim_io import stream_joint_path; stream_joint_path(Q)` with `Q` of shape (m, 7).

### Module 3 – IK multiplicity (expected output 5)
1. URDF + meshes as above; *IK solutions CSV* → `sim/data/m3_ik_x0.csv`
   (8 solutions of the tool-down pose at (0.55, 0.35, 0.60) m with x = 0; the branch label of
   row *i* is in `m3_ik_x0.csv.labels.txt`).
2. In *Target pose* type **x 0.55, y 0.35, z 0.60, roll 180, pitch 0, yaw 0** → *Apply target*.
   All 8 rows must show 0.000 mm / 0.0000°.
3. `m3_ik_multi_x.csv` holds the same world pose for x = −0.5, 0, +0.5 (20 solutions). The
   platform moves with each ghost, which shows directly how x changes the available branches.

⚠ Target angles in Module 3 use three.js Euler order **XYZ: R = Rx(roll)·Ry(pitch)·Rz(yaw)**, not
the URDF convention Rz·Ry·Rx. Convert any rotation with
`p6r.sim_io.module3_target_angles(R)`.

To export your own pose: `write_ik_solutions_csv('pose.csv', T_W, x)` (in `p6r/sim_io.py`).

### Module 4 – Redundancy & self-motion
1. URDF + meshes; *IK configurations CSV* → any 7-column file (the page starts from **row 2**,
   or row 1 if there is only one; e.g. `m3_ik_x0.csv` starts at branch `S+W+E-`).
2. *Kinematic selections → Redundancy joint* → **`base_x_joint`**; end-effector link `tool0`.
3. *Capture fixed pose*, then move the redundancy slider: the page prescribes x and solves the
   other six joints with its own damped least-squares IK to keep the pose. This is a
   numerical view of the 1-D self-motion in the proposal; our analytical result is
   "the same branch, re-solved at a shifted target".
4. Add the spherical obstacle to see clearance along the self-motion.

Notes: the slider label shows "°" for every joint; for `base_x_joint` the value is in metres.
The page's local solver can jump between branches near singularities. That is a useful
comparison with the branch-aware analysis, but it is not our IK.

### Module 5 – Path & joint-path execution (expected outputs 7–9)
1. URDF + meshes.
2. *Workspace path poses* → `sim/data/m5_workspace_path.csv` (columns `x,y,z,qw,qx,qy,qz`, world frame).
3. *Joint-space path* → `m5_joint_fixed_x0.csv` or `m5_joint_moving_base.csv`.
4. Play. White = prescribed path, red = FK of our joint path; they must overlap. The ghost
   shows the start configuration. No resampling is done, so write both files with the same `s` grid.

Write your own with `write_workspace_csv(path, T_path)` and `write_joint_csv(path, Q)`.
`check_joint_vs_workspace(Q, T_path)` reports the mismatch before you load anything.

The demo path (tool down, line from x = −0.6 to 1.2 m at y = 0.45 m with a 0.25 m arc in z) is
already a small instance of the project question:

| base | branches completing the path |
|---|---|
| fixed x = 0 | 4 of 8 (`S+W-E±`, `S-W+E±`) |
| ramp x(s) = −0.3 + 0.9 s | 8 of 8, and min σ(J_red) rises from 0.11 to 0.22 |

It is only an illustration of the file pipeline, not the planner of steps 5–6.

---

## 4. Playgrounds (no file upload of our robot)

- **D–H playground** (`dh_parameters.html`): only lists the course robots (custom 3R/6R, CuRo,
  iiwa 7, GoFa, IRB 4600, Puma 560, FANUC CRX-10iA/L, Enlight) and revolute joints. It uses
  the same standard D-H convention as `p6r` (θᵢ = qᵢ + θᵢ,₀, Rz Tz Tx Rx). Use it to rehearse
  frame assignment; the FANUC CRX also has a non-spherical wrist. Our UR12e table is
  validated numerically instead (`step1_fk_validation.py`).
- **Kinematic building blocks** (`building_blocks.html`): chain *D–H transformation* blocks with
  our `UR_DH` rows (plus a Translation block for x and the 0.41 m mount) to get the symbolic
  FK, or chain *Exponential* blocks with `S_HOME` for the PoE form. *Import Python* accepts a
  function that takes scalar arguments and returns a SymPy 3×3/3×1/4×4 matrix, so a symbolic
  version of `fk_dh` can be imported and compared block by block. Its *Quaternion ↔ RPY* block
  uses the URDF convention (Rz·Ry·Rx, radians), scalar-first quaternions like Module 5.

---

## 5. Pitfalls we hit (and fixed)

| Problem | Cause | Fix in this repo |
|---|---|---|
| UR links rotated by 90° when loading the original `.dae` files | UR Collada meshes declare `Z_UP`; three.js `ColladaLoader` (used by all modules) rotates them by −90° about X | `build_sim_assets.py` converts them to STL in the ROS frame (checked against the UR collision STLs) |
| UMR base invisible | simulators draw only `<mesh>` geometry, not `<box>` | base exported as `umr_base.stl` |
| Mesh not found | files are matched by **file name only** (lower case), folder ignored | unique names `ur12e_*.stl`, `umr_base.stl` |
| CSV rejected / rows skipped | rows must have exactly as many numbers as movable joints (7) | `write_joint_csv` checks shape, header row is ignored by the pages |
| Wrong EE readout | EE link defaults to the deepest leaf | choose `tool0` explicitly |
| Module 3 target "wrong" | Euler XYZ vs URDF RPY | `module3_target_angles(R)` |
| Collision mesh input | ignored by Modules 4/5 (no handler) | optional, can be left empty |

---

## 6. Verification record (20 Oct 2026)

- `verify_sim_assets.py`: sim URDF vs project URDF 0.0, vs D-H FK 6e-10 over 5000 random Q; joint
  order `[base_x, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`.
- Module 3 (TA page) with `m3_ik_x0.csv`: 8/8 solutions at 0.000 mm and 0.0000° for the target
  (0.55, 0.35, 0.60) m, roll 180°.
- Module 5 (TA page) with the moving-base path: FK readout at samples 0, 80, 200 =
  (−0.60, 0.45, 0.45), (0.12, 0.45, 0.688), (1.20, 0.45, 0.45) m, q = (0, 1, 0, 0), equal to the
  prescribed poses (the prismatic joint is handled correctly).
- WebSocket streamer: 201 `joint_state` messages with 7 values received by a test client.
