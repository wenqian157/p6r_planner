# Presentation prep: likely questions on sections 3–4, with answers

All numbers come from `drafts/ik_ur12e_full_output.txt` and `drafts/maps_section4_output.txt`.

## Analytical IK (section 3)

**Q1. Why not a Pieper decomposition?**
Pieper needs the last three axes to meet in one point (a spherical wrist), so that the wrist centre depends
only on q1..q3. On the UR the wrist offsets d5 = 0.120 m and d6 = 0.117 m keep axes 4, 5 and 6 apart.
Our 3J5 (section 2.2) shows this directly: the block C is not zero, because column q4 still moves O5 by d5.
The UR closed form uses a different simplification instead: axes 2, 3 and 4 are parallel.

**Q2. Why at most 8 solutions? A general 6R has up to 16.**
The parallel axes 2-3-4 make the problem decouple into three quadratic choices: q1 (side of the arm plane),
q5 (wrist flip) and q3 (elbow). That gives 2 × 2 × 2 = 8. A general 6R has no parallel axes, and its
univariate polynomial has degree 16. If the real UR limits of ±2π were used, each branch would have up to
2^6 angle representatives. The ±π limits are there so that each branch gives exactly one joint vector.

**Q3. Can a pose have an odd number of solutions?**
Not for regular poses. E+ and E- appear and disappear together (same condition |c3| ≤ 1), and so do S+
and S- (r ≥ d4). The round trip only ever gave 2, 4, 6 or 8 solutions. Odd counts only show up after the
singular class is applied, when one root of a pair is below the σ threshold.

**Q4. What does the solver do at a singularity?**
- Shoulder (r = d4) and elbow (|c3| = 1): the two roots coincide and both labels return the same joint
  vector. Small numerical overshoots are clipped.
- Wrist (sin q5 = 0): q4 and q6 turn about the same axis, so only q4 + q6 is determined. We set q6 = 0
  and q4 takes the whole angle. That point lies on a one-parameter family of solutions, not on a discrete
  set.

**Q5. How do you know it's correct and complete?**
- 200,000 random joint vectors were put through FK and then IK. The original was recovered in 100% of the
  regular samples (worst error 2.4e‑10 rad).
- Every returned root reproduces the pose through FK.
- In Section 3.5 all 8 roots were also checked against the separate URDF chain (error ~1e‑15 m).
- Completeness: the true configuration is always among the returned roots, and the counting argument (Q2)
  bounds how many there can be.

**Q6. Your labels are signs. Is E+ "elbow up"?**
Not in general. The labels are the signs in the formulas: S = the ± in q1, W = sign q5, E = sign q3.
Whether E+ looks "up" or "down" depends on the shoulder side (on the S− side the arm is mirrored). The
physical names are therefore S‑dependent. Sign labels are unambiguous, and they are what persists along a
path.

**Q7. Do the labels persist along a path?**
In 99.7% of about 195,000 steps, the nearest root at the next step had the same label. All 634 exceptions
were next to a branch-merging singularity:
- shoulder: 67%
- wrist: 24%
- elbow: 9%

At those points two branches coincide, so no labelling can separate them. This is why Section 5 tracks
solutions by continuity and checks σ_min, rather than following a fixed label.

**Q8. Where does the redundancy (x) show up in the IK?**
The IK never solves for x. x is a parameter: T06 = (Trans(x)·Mount·T_BASE)⁻¹ T* T_TOOL⁻¹. For a fixed
world pose, each branch traces a curve as x varies (the 1‑D self-motion). For the Section 1.4 pose:
- 4 branches exist for x ≥ 0.13 m;
- all 8 exist for x ≥ 0.38 m.

**Q9. Is it fast enough for the maps and paths?**
Yes: about 59,000 poses per second with all 8 branches, in plain numpy (no loop over poses). All the
Section 4 maps take about 7 s.

## Maps (section 4)

**Q10. If the maps are only shifted copies, why compute three?**
We checked the shift: map(+1) equals map(0) moved by 1 m on 100% of the overlapping cells. The useful
information is in the overlay. No single x covers the 3 m task strip (53–73%). Together the three cover
100%, and 27–33% of the strip is covered by exactly one x. Those cells are where the base position is
forced.

**Q11. How are the boundaries related to the Jacobian?**
We verified (error 2.5e‑16) that

det J_red = a2·a3·s3·s5·(a2c2 + a3c23 + d5s234)

and that the last factor satisfies f² + d4² = r². So the three IK conditions are exactly the zeros of
det J_red:
- r = d4 (shoulder)
- |c3| = 1 (elbow)
- s5 = 0 (wrist)

Every edge of the "no real IK" region is a place where two branches merge and J_red loses rank. These are
the critical curves from the lecture.

**Q12. Why is the joint-limit class empty?**
With ±π limits and every angle wrapped to (−π, π], no root can be outside the limits. We say so in the
report, and the class is still in the code. It would become non-empty with tighter, task-specific limits
(for example to avoid the base or cable wrap).

**Q13. Why is there no wrist singularity when the tool points down?**
Axis z1 (axes 2-3-4) is always horizontal. When the approach axis is vertical, w_z = z1·approach = 0,
so q5 = ±90° everywhere, which is as far from s5 = 0 as possible. With the tool horizontal, the approach
becomes parallel to z1 when the arm plane lines up with world X. That produces the two red wedges.

**Q14. Why do some branches reach 1.31 m and others only 1.08 m (tool down)?**
With q5 = ±90°, the wrist link d5 lies in the horizontal plane and points either away from axis 1 or
toward it. The 0.23 m difference is about 2·d5 = 0.24 m.

**Q15. Why does the slide rescue elbow singularities but not wrist ones?**
The slide column of J is [x̂; 0], a pure translation. At the elbow boundary the lost direction is radial
(a translation). It has an x component unless the boundary point lies along ±Y, so the slide restores
rank: 100% of elbow cells are rescued. At the wrist singularity the lost direction is mostly a rotation,
which a prismatic joint cannot produce: only 11% are rescued. The σ_min(J) map shows this by direction:
rescue is strong along ±X and weak along ±Y. That is Type A versus Type B from Section 2.3, seen across
the whole map.

**Q16. σ_min mixes metres and radians. Isn't the 0.05 threshold arbitrary?**
Partly, yes, and we say so. The sensitivity check shows that 0.01 / 0.02 / 0.05 / 0.10 mark 0.0 / 0.2 /
2.1 / 7.4% of reachable cells. The singular class is always a thin band around the same analytic critical
curves. A unit-consistent alternative is to scale the angular rows by a characteristic length, such as the
reach (about 1.2 m).

**Q17. What is not modelled?**
- Collisions: the arm with the UMR body or the ground, and self-collision. Cells near the base box may need
  configurations that pass through the deck.
- Joint velocity and acceleration limits.
- The base moves only in X (by the project definition; the real UMR is omnidirectional).
- The mount position is a placeholder (`umr_params.py` says it should be measured).

**Q18. Why z = 0.60 m, and why +Y for the horizontal tool?**
z = 0.60 m is about shoulder height (0.42 + 0.18), where the reach is largest. +Y points the tool toward
the task strip beside the path. Rotating the approach about the vertical rotates the whole map about
axis 1 (+X approach gave the same areas as +Y). Branch areas hardly change with height (checked at 0.3 and
0.9 m).

## Open points to align on in the team

1. **Mount height.** `p6r/kinematics.py` and the older `p6r/ik.py` use 0.41 m. Her scripts and report use
   0.42 m (0.41 deck + 0.01 adapter plate). We should pick one code path.
2. **End frame.** Her chain ends in the ROS `flange` frame, whose x axis points out of the tool. The old
   `p6r` package and `step3_ik_validation.py` define "tool down" with z out (tool0). Any orientation reused
   from the old code needs converting.
3. **Task region and path for Section 5.** I used a 3 m strip at y = 0.5–0.9 m. The Section 5 path should
   go through the cells only one x can cover.
