# Step 4B: subchain Jacobian 3J5 and det A * det W

Script: `scripts/step04b_3J5.py`

```text
x = 0.5 m,  arm q = [10, -60, 90, -120, -90, 30] deg,  velocity point O5,  frame = UR DH frame 3

3J5 =
               q1       q2       q3   |       q4       q5       q6
  vx       0.1508  -0.6726  -0.0599   |  -0.0599   0.0000  -0.0000
  vy      -0.0871  -0.6753  -0.6753   |  -0.1038   0.0000   0.0000
  vz       0.9212  -0.0000   0.0000   |  -0.0000   0.0000  -0.0000
       -----------------------------+---------------------------
  wx       0.5000  -0.0000  -0.0000   |  -0.0000  -0.8660  -0.5000
  wy       0.8660   0.0000   0.0000   |   0.0000   0.5000  -0.8660
  wz       0.0000   1.0000   1.0000   |   1.0000  -0.0000  -0.0000

           [   A   |   C   ]
           [   B   |   W   ]

det A           =    0.3812      (A = 3J5[0:3, 0:3], how q1..q3 move O5)
det W           =    1.0000      (W = 3J5[3:6, 3:6], how q4..q6 rotate the tool)
det A * det W   =    0.3812
det 3J5         =    0.3226
det 0J_red      =    0.3226      (same as det 3J5: point shift and rotation have unit det)

C = 3J5[0:3, 3:6] column norms = [0.1199 0.     0.    ]
C is not 0: column q4 still moves O5 by 0.1199 m per rad (= d5, the wrist offset).
So 3J5 is NOT block triangular and det 3J5 = det A * det(W - B A^-1 C), not det A * det W.
check:  det A * det(W - B A^-1 C) =    0.3226
```
