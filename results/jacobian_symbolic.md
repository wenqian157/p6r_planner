# Symbolic Jacobians of the P6R: 0J6 (J_red) and the full 6 x 7 J

Script: `scripts/jacobian_symbolic.py`

```text
=== 1. 0J6: arm Jacobian in DH frame 0 of the arm, velocity point = flange, rows [vx vy vz wx wy wz] ===
  column q1:
     vx: -a2*sin(q1)*cos(q2) - a3*sin(q1)*cos(q2 + q3) + d4*cos(q1) - d5*sin(q1)*sin(q2 + q3 + q4) + d6*sin(q1)*sin(q5)*cos(q2 + q3 + q4) + d6*cos(q1)*cos(q5)
     vy: a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2 + q3) + d4*sin(q1) + d5*sin(q2 + q3 + q4)*cos(q1) + d6*sin(q1)*cos(q5) - d6*sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     vz: 0
     wx: 0
     wy: 0
     wz: 1
  column q2:
     vx: (-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: (-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a2*cos(q2) + a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: sin(q1)
     wy: -cos(q1)
     wz: 0
  column q3:
     vx: (-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: (-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: sin(q1)
     wy: -cos(q1)
     wz: 0
  column q4:
     vx: (d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: (d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: sin(q1)
     wy: -cos(q1)
     wz: 0
  column q5:
     vx: -d6*(sin(q1)*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3 + q4))
     vy: d6*(-sin(q1)*cos(q5)*cos(q2 + q3 + q4) + sin(q5)*cos(q1))
     vz: -d6*sin(q2 + q3 + q4)*cos(q5)
     wx: sin(q2 + q3 + q4)*cos(q1)
     wy: sin(q1)*sin(q2 + q3 + q4)
     wz: -cos(q2 + q3 + q4)
  column q6:
     vx: 0
     vy: 0
     vz: 0
     wx: sin(q1)*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     wy: -sin(q1)*sin(q5)*cos(q2 + q3 + q4) - cos(q1)*cos(q5)
     wz: -sin(q5)*sin(q2 + q3 + q4)

  free symbols: ['a2', 'a3', 'd4', 'd5', 'd6', 'q1', 'q2', 'q3', 'q4', 'q5']
  -> q6 absent: the last joint moves no axis and no point before the flange.
  -> d1 absent: it only lifts frame 0 and the flange together.
  -> q1 appears in every column: it rotates all columns together, so det does not depend on it.

=== 2. WJ6 = J_red: the same arm Jacobian in the world frame, velocity point = flange ===
  WJ6 = blockdiag(R0, R0) 0J6,  R0 = Rz(pi) = rotation of DH frame 0 in the world (base_link_inertia yaw)
  -> every column has its rows vx, vy, wx, wy negated; vz, wz unchanged.
  column q1:
     vx: a2*sin(q1)*cos(q2) + a3*sin(q1)*cos(q2 + q3) - d4*cos(q1) + d5*sin(q1)*sin(q2 + q3 + q4) - d6*sin(q1)*sin(q5)*cos(q2 + q3 + q4) - d6*cos(q1)*cos(q5)
     vy: -a2*cos(q1)*cos(q2) - a3*cos(q1)*cos(q2 + q3) - d4*sin(q1) - d5*sin(q2 + q3 + q4)*cos(q1) - d6*sin(q1)*cos(q5) + d6*sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     vz: 0
     wx: 0
     wy: 0
     wz: 1
  column q2:
     vx: -(-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a2*cos(q2) + a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q3:
     vx: -(-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q4:
     vx: -(d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q5:
     vx: d6*(sin(q1)*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3 + q4))
     vy: -d6*(-sin(q1)*cos(q5)*cos(q2 + q3 + q4) + sin(q5)*cos(q1))
     vz: -d6*sin(q2 + q3 + q4)*cos(q5)
     wx: -sin(q2 + q3 + q4)*cos(q1)
     wy: -sin(q1)*sin(q2 + q3 + q4)
     wz: -cos(q2 + q3 + q4)
  column q6:
     vx: 0
     vy: 0
     vz: 0
     wx: -sin(q1)*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     wy: sin(q1)*sin(q5)*cos(q2 + q3 + q4) + cos(q1)*cos(q5)
     wz: -sin(q5)*sin(q2 + q3 + q4)

=== 3. WJ: whole-body Jacobian (6 x 7) in the world frame, columns (x, q1..q6) ===
  WJ = [ e | WJ6 ],   e = [1 0 0 0 0 0]^T: the slide column is pure x translation and contains no q.
  J_red = WJ[:, 1:] = WJ6 (x fixed).
  column x:
     vx: 1
     vy: 0
     vz: 0
     wx: 0
     wy: 0
     wz: 0
  column q1:
     vx: a2*sin(q1)*cos(q2) + a3*sin(q1)*cos(q2 + q3) - d4*cos(q1) + d5*sin(q1)*sin(q2 + q3 + q4) - d6*sin(q1)*sin(q5)*cos(q2 + q3 + q4) - d6*cos(q1)*cos(q5)
     vy: -a2*cos(q1)*cos(q2) - a3*cos(q1)*cos(q2 + q3) - d4*sin(q1) - d5*sin(q2 + q3 + q4)*cos(q1) - d6*sin(q1)*cos(q5) + d6*sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     vz: 0
     wx: 0
     wy: 0
     wz: 1
  column q2:
     vx: -(-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(-a2*sin(q2) - a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a2*cos(q2) + a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q3:
     vx: -(-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(-a3*sin(q2 + q3) + d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q4:
     vx: -(d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*cos(q1)
     vy: -(d5*cos(q2 + q3 + q4) + d6*sin(q5)*sin(q2 + q3 + q4))*sin(q1)
     vz: d5*sin(q2 + q3 + q4) - d6*sin(q5)*cos(q2 + q3 + q4)
     wx: -sin(q1)
     wy: cos(q1)
     wz: 0
  column q5:
     vx: d6*(sin(q1)*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3 + q4))
     vy: -d6*(-sin(q1)*cos(q5)*cos(q2 + q3 + q4) + sin(q5)*cos(q1))
     vz: -d6*sin(q2 + q3 + q4)*cos(q5)
     wx: -sin(q2 + q3 + q4)*cos(q1)
     wy: -sin(q1)*sin(q2 + q3 + q4)
     wz: -cos(q2 + q3 + q4)
  column q6:
     vx: 0
     vy: 0
     vz: 0
     wx: -sin(q1)*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3 + q4)
     wy: sin(q1)*sin(q5)*cos(q2 + q3 + q4) + cos(q1)*cos(q5)
     wz: -sin(q5)*sin(q2 + q3 + q4)

=== 4. det J_red = det 0J6 = det WJ6 (factored) ===
  det J_red = a2*a3*(a2*cos(q2) + a3*cos(q2 + q3) + d5*sin(q2 + q3 + q4))*sin(q3)*sin(q5)    [24 s]

  reading: det J_red = a2 a3 sin(q3) sin(q5) (a2 cos q2 + a3 cos(q2+q3) + d5 sin(q2+q3+q4))
     sin q3 = 0                     elbow family    (q3 = 0, pi)
     sin q5 = 0                     wrist family    (q5 = 0, pi)
     a2 c2 + a3 c23 + d5 s234 = 0   shoulder family (in-plane radial coordinate of O5 is zero)
  det does not contain q1, q6, d1, d4, d6: the singular set lives in (q2, q3, q4, q5), as observed numerically.
  Compare lecture-5 slide 48 (ABB IRB 4600, intersecting wrist):
     det = -a2 (a3 s3 + d4 c3)(a1 + a2 s2 + a3 s_psi + d4 c_psi) s5   -> same wrist factor s5.

=== numeric checks, 200 random (x, q), UR12e lengths, against jacobian_p6r.py ===
  max |blockdiag(R0,R0) 0J6 - J_red_numeric| = 8.9e-16
  max |J_full - J_numeric (6 x 7)|           = 8.9e-16
  max |det J_red symbolic - numeric|         = 4.4e-16
  shoulder factor at the step-4C configuration q = [0, 55, 90, 111.95, -90, 0] deg: 1.4e-06

total time 31 s
```
