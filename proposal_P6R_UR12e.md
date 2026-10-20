# ENG-654 | Kinematics-Grounded Motion Planning for Robots

## Project proposal

# Base Position as a Redundancy Parameter: Branch-Resolved IK, Workspace Structure and Path Planning for a Mobile Base + UR12e

## Motivation

An AgileX UMR omnidirectional base can translate in X and Y and rotate in yaw. It carries a
6R UR12e arm performing a full six-dimensional Cartesian task. Taken as one system the robot
is redundant: a single tool pose is realised by a continuous family of configurations rather
than a discrete set.

For this project the base is restricted to translation along the world X axis and treated as a
prismatic joint. The result is a P6R robot (one prismatic joint followed by six revolute
joints) with seven degrees of freedom and a one-dimensional self-motion for every reachable
tool pose. Fixing the base position x = a reduces the robot to a UR12e whose target has been
shifted by -a along X. Every value of a therefore produces the same finite set of IK branches,
but applied to a different target. The redundancy becomes visible only when x is allowed to
vary while the task stays fixed: the base position then decides which branches exist, how far
each is from a joint limit or a singularity, and whether a continuous execution is possible.

The central question of this project is:

> **How does the base position x select among the IK branches and reachable regions of the
> UR12e, and how can x and the branch be planned jointly so that a Cartesian task is executed
> continuously?**

## Task

### Kinematic model (DH, PoE, screws)

No course asset exists for the UR12e or the UMR. Assemble a URDF of the whole body from the
Universal Robots description package (UR12e, with UR10e as a fallback if the 12e files are not
available) and the mounting geometry of the AgileX UMR, with the base translation modelled as a
prismatic joint along world X. From this URDF identify the seven joint axes, the fixed offsets,
the joint limits, the base frame and the flange frame. The physical UR joints span +/- 2 pi;
for this project the joint limits are taken as +/- pi on every joint, so that after wrapping each
geometric branch yields exactly one candidate joint vector.

Build a consistent DH model and determine the seven screw axes in a clearly defined home
configuration. Derive the forward kinematics with both the DH and the Product-of-Exponentials
formulations and show numerically that both agree with the URDF model at representative
configurations.

### Jacobians

From the screw representation construct the complete 6 x 7 geometric Jacobian of the P6R
robot. Fix x = a and derive the reduced 6 x 6 Jacobian of the arm alone. Derive the
wrist/subchain Jacobian 3J5 used in the course and interpret its columns geometrically.

Use the full and reduced Jacobians to distinguish a true singularity of the redundant robot from
a singularity of the fixed-x representation. The prismatic column is a pure translation along X,
so the full Jacobian loses rank only when the reduced Jacobian is singular and the lost
direction has no X component. The expected result, to be verified numerically, is that a wrist
singularity (loss of a rotation direction) remains a singularity of the whole body, while
shoulder and elbow singularities (loss of a translation direction) are partly recoverable by the
base.

### Analytical IK solver

The inverse kinematics is a central part of this project and must be derived from the robot
geometry. For a fixed value of x, derive a complete analytical IK formulation that exposes all
regular discrete solution branches of the resulting 6R problem. The UR wrist is offset, so the
derivation follows the UR-specific closed form (joints 1 and 5 from the wrist geometry, a planar
2R for joints 2 and 3, then joints 4 and 6) rather than a Pieper decomposition.

The implementation must return all real solutions, wrap every angle to (-pi, pi], apply the
+/- pi joint limits adopted for this project, and verify
every retained solution through the independent forward model. Label each solution by its
shoulder (left/right), elbow (up/down) and wrist (flip/no flip) choice, giving up to eight
branches, and verify that the labels persist between neighbouring task poses. Validate the
solver by a round trip: sample random joint vectors within limits, compute the pose, solve, and
check that the original configuration is recovered and that every returned configuration
reproduces the pose. Vectorise the solver over arrays of target poses from the start, since the
maps and path experiments below call it millions of times.

### Reachable-position maps over discrete base positions

Once the IK is available, use it to construct branch-resolved reachable-position maps. Fix the
flange orientation (two orientations: tool pointing down, as in picking, and tool horizontal)
and sweep the flange position over a two-dimensional grid in the world frame. At each grid point
record, per branch, one of four outcomes: feasible, no real IK, joint-limit rejection, or loss of
regularity (smallest singular value of the reduced Jacobian below a threshold).

Repeat the maps for x = -1 m, 0 m and +1 m and overlay them in the world frame for a fixed task
region. Because fixing x only translates the arm, the individual maps are shifted copies of one
another; the content lies in their overlaps and gaps, which show which (x, branch) pairs cover
which world positions. Relate the boundaries of the feasible regions to the reduced and full
Jacobians (critical curves, in the sense of the IKS-distribution lecture) rather than treating
the workspace as a purely numerical point cloud.

### Cartesian path and the (s, x) redundancy plane

Select a pick-and-place path whose span exceeds the arm's comfortable reach, so that the base
position matters, and that passes through an informative region of the workspace maps.
Parametrise the path by s in [0, 1].

1. **Fixed-x baseline.** For x = 0 m, start a continuation from every admissible IK solution at
   s = 0 and follow each branch separately along the path. Record where each branch completes,
   loses IK, hits a joint limit, loses continuity, or drops below the singularity margin. Repeat
   for one other value of x.

2. **Redundancy plane.** For each branch, evaluate the four-class outcome over a two-dimensional
   grid in (s, x). A fixed-x execution is a horizontal line through this plane; a continuous
   execution with a moving base is any continuous curve through a feasible region. The base
   velocity limit becomes a bound on the slope dx/ds.

3. **Planning comparison.** Compare a greedy continuation (nearest configuration at each step,
   x chosen locally) with a branch-aware planner that keeps all branch alternatives and searches
   the (s, x) plane for a globally consistent, slope-bounded curve. Explain the difference in
   outcome using the (s, x) maps and the singularity analysis.

## Expected output

1. A validated DH model, home configuration and seven screw axes for the P6R whole body,
   together with the assembled URDF.
2. Numerical agreement between URDF, DH and PoE forward kinematics.
3. The complete 6 x 7 screw Jacobian, the reduced fixed-x 6 x 6 Jacobian, the subchain Jacobian
   3J5, and a numerical demonstration of which arm singularities remain singularities of the
   whole body.
4. A complete analytical fixed-x IK solver with explicit branch labels, periodic-angle and
   joint-limit handling, and round-trip FK verification.
5. Visualisations of all IK solutions for one representative tool pose.
6. Branch-resolved reachable-position maps for two flange orientations and x = -1, 0, +1 m,
   with the four failure classes separated, and their world-frame overlay.
7. Per-branch continuation plots for the fixed-x pick-and-place path, showing completion or the
   precise failure location.
8. The (s, x) redundancy-plane maps for every branch, with the planned base trajectory x(s)
   drawn on them.
9. A comparison between greedy continuation and the branch-aware planner, with joint
   trajectories, joint-limit margins, Cartesian residuals and singularity margins for the
   representative paths.
