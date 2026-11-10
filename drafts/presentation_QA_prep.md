# Presentation prep: likely questions, with answers (sections 3–5)

The numbers are those of `results/Report_merged.md`. Section 4 uses the maps at z = 0.9 m, like 4.1.

## Analytical IK (section 3)

**Q1. Why not a Pieper decomposition?**
Pieper needs the last three axes to meet in one point, so that the wrist centre depends only on q1..q3. On the
UR the wrist offsets d5 = 0.120 m and d6 = 0.117 m keep axes 4, 5 and 6 apart. What the UR has instead is three
parallel axes (2, 3, 4). That gives the "plane split" of 2.2: 3J5 is block triangular with blocks (P, rho, W2),
and those three blocks are the three groups of the IK.

**Q2. Why at most 8 solutions? A general 6R has up to 16.**
The parallel axes 2-3-4 decouple the problem into three independent ± choices: q1 (which side of the arm
plane), q5 (wrist flip) and q3 (elbow). That gives 2 × 2 × 2 = 8. A general 6R leads to a polynomial of degree
16. With the real UR limits of ±2π, each branch would have up to 2^6 angle representatives. The ±π convention
gives exactly one per branch.

**Q3. Where is O5 and why start there?**
O5 is where the wrist-2 and wrist-3 axes cross, d6 behind the flange on the tool axis. It is fixed by the target
alone (p − d6·z6). Its sideways offset from the upper-arm plane is always d4, whatever q2..q5 are. So it
constrains q1 only: two tangents from axis 1, hence S±.

**Q4. Can a pose have an odd number of solutions?**
Not for regular poses. Roots appear and disappear in pairs at the singular surfaces. The validation only ever
found 2, 4, 6 or 8 solutions.

**Q5. What does the solver do at a singularity?**
- Shoulder (rho = 0, r = d4) and elbow (sin q3 = 0): the two roots coincide, so two labels give one
  configuration.
- Wrist (sin q5 = 0): q4 and q6 turn about the same axis, only q4 + q6 is fixed, and the solver sets q6 = 0.
  That is a one-parameter family, not a discrete set.

**Q6. How do you know it's correct and complete?**
- 2000 random configurations: the input is always recovered (6.4e-11 rad), and every root reaches the pose
  (3.4e-12).
- The label read off each solution matches its sign triple every time (0 mismatches out of 14 426 solutions).
- The vectorised version passes the same round trip on 200 000 samples (2.4e-10 rad).
- Module 3 (independent FK): all 8 roots match the target.
- Completeness comes from the counting argument in Q2.

**Q7. Is E+ "elbow up"?**
Not in general. The labels are the sign triple (sign rho, sign q5, sign q3). Physical "up/down" flips with the
shoulder side. Sign labels are unambiguous, and they are what persists along a path.

**Q8. Do labels persist along a path?**
In 99.72 % of about 194 000 steps. The 545 switches were all next to a singular surface: shoulder 68 %, wrist
24 %, elbow 8 %. So the planner tracks by continuity and checks sigma_min, rather than following a fixed label.

**Q9. Why is the Module 3 error 0.055 mm and not 0?**
The target was typed with 4 decimals. The rounding of (1.376941, 0.331465, 0.728989) m is exactly 0.055 mm, the
same for every row, so it is not an IK error. The orientation error is 0.0000°.

**Q10. Are all 8 solutions usable?**
No. Two of them (4 and 5) pass through the UMR deck, and 2 and 7 clear it by a few centimetres. That is a rough
check by distance to the deck, not a real collision checker. Collisions are outside this project.

## Maps (section 4)

**Q11. If the maps are only shifted copies, why compute three?**
The shift is exact (100 % of overlapping cells). The useful information is in the overlay. On the 3 m task strip,
one base position covers 50–68 %, the three together cover 100 %, and about 34 % is covered by one x only. That
is where the base position is forced.

**Q12. How are the boundaries related to the Jacobian?**
det J_red = a2 a3 sin q3 sin q5 rho (2.2, verified numerically to 2.5e-16), and rho² + d4² = r². So the IK
conditions r = d4, |cos q3| = 1 and sin q5 = 0 are the zeros of det J_red. Every edge of a "no real IK" region
is where two branches merge.

**Q13. Why is the joint-limit class empty?**
With ±π limits and wrapped angles, no root can leave the limits. The limit does act along a path, as the ±π seam
(section 5).

**Q14. Why no wrist singularity with the tool pointing down?**
The axes 2-3-4 are always horizontal. With a vertical tool axis, cos q5 = 0, so q5 = ±90°. With the tool
horizontal, the tool axis becomes parallel to axes 2-3-4 when the arm plane runs along world X: those are the two
red wedges.

**Q15. Why do some branches reach about 0.24 m farther?**
With the tool down, q5 = ±90°, so the wrist link d5 lies horizontal and points away from or toward axis 1. The
difference is about 2·d5. This is the 4-branch ring of 4.2.

**Q16. Why does the slide rescue elbow singularities but not wrist ones?**
The slide column is a pure translation along X.
- Elbow boundary: the lost direction is radial. The slide supplies it unless the point lies along ±Y
  (88–96 % of the cells are rescued).
- Wrist: the lost direction is mostly a rotation, which a prismatic joint cannot produce (12.5 % rescued).

This is type A vs type B of 2.3, seen across the whole map.

**Q17. sigma_min mixes metres and radians. Isn't 0.05 arbitrary?**
Partly, yes. Thresholds of 0.01 / 0.02 / 0.05 / 0.10 mark 0.0 / 0.3 / 1.8 / 6.9 % of reachable cells, always as
a thin band around the same analytic curves. A unit-consistent alternative is to scale the angular rows by a
characteristic length.

## Path and planning (section 5)

**Q18. What is the (s, x) map?**
For each path point s and base position x, we run the analytical IK and colour each branch green if it has a
regular root. A parked base is a horizontal line through the map. A moving base is any curve through green whose
slope dx/ds respects the base speed. The redundancy becomes a 2-D picture per branch.

**Q19. Does the base really have to move?**
From the start x(0) = −1 m, yes: every branch stops by s = 0.24 because the tool leaves the reach. A base parked
at x ∈ [−0.10, 0.10] m could do it on three branches, but it would have to be there already.

**Q20. Why do two branches have no schedule even though they are mostly green?**
Along the path one of their joints (q4, or q2) must pass through 180°. Each sample is inside ±π, so the map
shows nothing wrong, but a continuous motion cannot cross the seam. The joint-limit convention therefore acts as a
path constraint. With ±2π limits these branches could become feasible (not checked).

**Q21. What does the DP planner optimise, and under which constraints?**
- Nodes are admissible (s, x) points on one branch.
- Edges connect neighbouring s with |dx| ≤ 0.05 m (base speed ≤ 1.25 × tool speed) and |dq| ≤ 0.5 rad on
  unwrapped angles (no seam crossing).
- Cost = joint travel + 2 × base travel.
- DP returns the cheapest complete schedule from x(0) = −1 m: 6 of 8 branches, best S+ W− E+, cost 10.31.

**Q22. Why not just use a greedy planner?**
- With a fast base, greedy completes the same 6 branches but costs 10–46 % more. The min-cost rule hugs the reach
  boundary (sigma_min 0.033) and the max-margin rule moves the base 3.9 m instead of 1.25 m.
- With a base slower than about 0.6 × the tool speed, greedy fails where DP succeeds. At 0.42 ×, DP completes 3
  branches and greedy none.
- The (s, x) map shows why: the base must start climbing early to pass above a grey wedge at s ≈ 0.5, and a
  one-step rule cannot see it coming.

**Q23. Is the executed motion accurate?**
Every executed configuration is a closed-form IK root, re-checked through the URDF chain: the Cartesian residual
is about 1e-15 m. Only the motion between samples (4 cm apart) is unchecked.

**Q24. Where do singularities enter the planning?**
On this path, hardly at all. Tool-down removes the wrist family, and the rectangle avoids the shoulder hole.
Branches die from the reach boundary (elbow) and the ±π seam. sigma_min(J) stays ≥ 0.17 even when the arm alone
drops to 0.033, because the slide supplies the lost radial direction (type A).

**Q25. What would you do next?**
- A collision check against the UMR body (removes solutions 4 and 5 in 3.7).
- The real ±2π limits.
- The full omnidirectional base (x, y, yaw), which gives a 3-D redundancy space instead of a line.
- A time-parametrised planner with base velocity and acceleration limits.

## Points to align on in the team

1. Does `Report_merged.md` replace `Report.md`?
2. The note "(the final chart will hit the a wall)" at the top of section 5: is a wall obstacle planned? If not,
   delete the line.
3. Code referenced from `drafts/`: move it into `scripts/` with step numbers?
4. The old `p6r/` package, `step1_fk_validation.py`, `step3_ik_validation.py` and `sim/umr_ur12e_sim.urdf` use a
   0.41 m mount and a different base yaw: delete them or mark them as superseded.
