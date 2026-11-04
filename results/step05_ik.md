# Step 5: analytical IK of the UR12e, validation and branches

Script: `scripts/step05_ik.py`, solver: `scripts/ik_ur12e.py`

```text
=== 1. validation on 2000 random configurations q (joint limits +/- pi) ===
  max |q_true - nearest branch|  = 6.4e-11 rad   (the input configuration is always recovered)
  max |FK(branch) - T06|         = 3.4e-12       (every returned branch reaches the pose)
  number of branches per pose    = {2: 51, 4: 268, 6: 98, 8: 1583}
  (fewer than 8: the other shoulder or wrist choice puts O4 out of the planar 2R reach, |cos q3| > 1)

=== 2. all branches of one pose: T06 = FK(q0), q0 = [  10.  -60.   90. -120.  -90.   30.] deg ===
  branch           q1       q2       q3       q4       q5       q6       rho  sin q3  sin q5   det J_red    |FK-T|
  S+ W+ E+      10.00   -37.74    50.91    76.83    90.00  -150.00    -0.921   0.776   1.000  -2.504e-01   1.5e-15
  S+ W+ E-      10.00    11.27   -50.91   129.64    90.00  -150.00    -0.921  -0.776   1.000   2.504e-01   1.9e-15
  S+ W- E+      10.00   -60.00    90.00  -120.00   -90.00    30.00    -0.921   1.000  -1.000   3.226e-01   1.5e-15
  S+ W- E-      10.00    26.02   -90.00   -26.02   -90.00    30.00    -0.921  -1.000  -1.000  -3.226e-01   1.5e-15
  S- W+ E+    -148.59   153.98    90.00  -153.98    90.00    51.41     0.921   1.000   1.000   3.226e-01   1.3e-15
  S- W+ E-    -148.59  -120.00   -90.00   -60.00    90.00    51.41     0.921  -1.000   1.000  -3.226e-01   1.3e-15
  S- W- E+    -148.59   168.73    50.91    50.36   -90.00  -128.59     0.921   0.776  -1.000  -2.504e-01   1.3e-15
  S- W- E-    -148.59  -142.26   -50.91   103.17   -90.00  -128.59     0.921  -0.776  -1.000   2.504e-01   1.3e-15
  reading: the three sign columns are the branch label; every branch has a different sign triple,
           so each one lives in its own aspect (region cut out by the three singular surfaces).

=== 3. branch label vs sign triple read from the joint vector ===
  14426 solutions over the 2000 random poses, label mismatches: 0
  -> the label is a function of q alone: (sign rho, sign q5, sign q3).  Consistent with the UR being
     non-cuspidal: one solution per aspect, and a change of branch needs a crossing of det J_red = 0.

=== 4. the three singular families: branches merge in pairs ===

  elbow    sin q3 = 0:  q = [0, -45, 0, -45, -90, 0] deg,  det J_red = 4.6e-17
  branch           q1       q2       q3       q4       q5       q6       rho  sin q3  sin q5   det J_red    |FK-T|
  S+ W- E+       0.00   -45.00     0.00   -45.00   -90.00     0.00    -0.957   0.000  -1.000   4.604e-17   2.2e-16
  S+ W- E-       0.00   -45.00     0.00   -45.00   -90.00     0.00    -0.957   0.000  -1.000   4.604e-17   2.2e-16
  S- W+ E+    -159.38  -135.00     0.00  -135.00    90.00    20.62     0.957   0.000   1.000   9.990e-09   1.1e-15
  S- W+ E-    -159.38  -135.00    -0.00  -135.00    90.00    20.62     0.957  -0.000   1.000  -9.990e-09   9.7e-16
  identical joint vectors among the branches: 2 pair(s)  -> two labels, one configuration: the branch boundary

  wrist    sin q5 = 0:  q = [10, -60, 90, -120, 0, 30] deg,  det J_red = 4.1e-17
  branch           q1       q2       q3       q4       q5       q6       rho  sin q3  sin q5   det J_red    |FK-T|
  S+ W+ E+      10.00   -61.21    85.17   -83.97     0.00     0.00    -0.921   0.996   0.000   3.929e-17   5.0e-16   (wrist singular, q6 set to 0)
  S+ W+ E-      10.00    20.31   -85.17     4.86     0.00     0.00    -0.921  -0.996   0.000  -4.288e-17   1.9e-16   (wrist singular, q6 set to 0)
  S+ W- E+      10.00   -61.21    85.17   -83.97    -0.00     0.00    -0.921   0.996  -0.000   3.929e-17   5.0e-16   (wrist singular, q6 set to 0)
  S+ W- E-      10.00    20.31   -85.17     4.86    -0.00     0.00    -0.921  -0.996  -0.000  -4.288e-17   1.9e-16   (wrist singular, q6 set to 0)
  S- W+ E+    -148.59   150.99    76.62   -47.61   158.59   120.00     0.921   0.973   0.365   1.146e-01   3.2e-15
  S- W+ E-    -148.59  -135.54   -76.62    32.15   158.59   120.00     0.921  -0.973   0.365  -1.146e-01   3.4e-15
  S- W- E+    -148.59   169.66    66.48   123.86  -158.59   -60.00     0.921   0.917  -0.365  -1.080e-01   3.2e-15
  S- W- E-    -148.59  -126.47   -66.48  -167.05  -158.59   -60.00     0.921  -0.917  -0.365   1.080e-01   3.3e-15
  identical joint vectors among the branches: 2 pair(s)  -> two labels, one configuration: the branch boundary

  shoulder rho = 0   :  q = [0, 55, 90, 111.9529, -90, 0] deg,  det J_red = -6.4e-09
  branch           q1       q2       q3       q4       q5       q6       rho  sin q3  sin q5   det J_red    |FK-T|
  S+ W+ E+       0.00    35.13    97.12   -55.30    90.00  -180.00    -0.000   0.992   1.000  -6.565e-09   3.3e-16
  S+ W+ E-       0.00   127.75   -97.12    46.33    90.00  -180.00    -0.000  -0.992   1.000   6.565e-09   6.7e-16
  S+ W- E+       0.00    55.00    90.00   111.95   -90.00     0.00    -0.000   1.000  -1.000   6.616e-09   5.1e-16
  S+ W- E-       0.00   141.02   -90.00  -154.07   -90.00     0.00    -0.000  -1.000  -1.000  -6.616e-09   4.7e-16
  S- W+ E+      -0.00    35.13    97.12   -55.30    90.00   180.00     0.000   0.992   1.000   6.565e-09   3.1e-16
  S- W+ E-      -0.00   127.75   -97.12    46.33    90.00   180.00     0.000  -0.992   1.000  -6.565e-09   3.1e-16
  S- W- E+      -0.00    55.00    90.00   111.95   -90.00    -0.00     0.000   1.000  -1.000  -6.616e-09   6.9e-16
  S- W- E-      -0.00   141.02   -90.00  -154.07   -90.00    -0.00     0.000  -1.000  -1.000   6.616e-09   4.7e-16
  identical joint vectors among the branches: 4 pair(s)  -> two labels, one configuration: the branch boundary

=== 5. one world pose (from x = 0.5 m, q0 above), IK of the arm for several base positions x ===
   x [m] branches   labels
   -0.40        0   
   -0.30        0   
   -0.20        0   
   -0.10        0   
   -0.00        0   
    0.10        0   
    0.20        4   S+W-E+ S+W-E- S-W+E+ S-W+E-
    0.30        4   S+W-E+ S+W-E- S-W+E+ S-W+E-
    0.40        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    0.50        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    0.60        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    0.70        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    0.80        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    0.90        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    1.00        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    1.10        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    1.20        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    1.30        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
    1.40        8   S+W+E+ S+W+E- S+W-E+ S+W-E- S-W+E+ S-W+E- S-W-E+ S-W-E-
  reading: the base position selects which branches exist.  This row of counts is one column of the
           (s, x) plane: a fixed path point s, x swept.
```
