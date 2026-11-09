# Base Position as a Redundancy Parameter: Branch-Resolved IK, Workspace Structure and Path Planning for a Mobile Base + UR12e

ENG-654 | Kinematics-Grounded Motion Planning for Robots | Project 1G Report

<img src="figures/ur12e.jpeg" height="250">
<img src="figures/umr.png" height="250">

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

## Methods

### 1. Forward Kinematic models of P6R
First step, 3 models: URDF chain, DH table, PoE from screws are built with the UR12e URDF sheet. Secondly, the base UMR data is added to each model. 

#### 1.1 URDF chain from URDF sheet and AgileX UMR product sheet:
```python 
UR_CHAIN = [
    ("base_link -> base_link_inertia", [0, 0, 0],              [0, 0, pi],        None),
    ("1 shoulder_pan",                 [0, 0, 0.1807],         [0, 0, 0],         [0, 0, 1]),
    ("2 shoulder_lift",                [0, 0, 0],              [pi/2, 0, 0],      [0, 0, 1]),
    ("3 elbow",                        [-0.6127, 0, 0],        [0, 0, 0],         [0, 0, 1]),
    ("4 wrist_1",                      [-0.57155, 0, 0.17415], [0, 0, 0],         [0, 0, 1]),
    ("5 wrist_2",                      [0, -0.11985, 0],       [pi/2, 0, 0],      [0, 0, 1]),
    ("6 wrist_3",                      [0, 0.11655, 0],        [pi/2, pi, pi],    [0, 0, 1]),
    ("wrist_3 -> flange",              [0, 0, 0],              [0, -pi/2, -pi/2], None),
]
P6R_CHAIN = [
    ("0 base_x  world -> umr_base",   [0, 0, 0],      [0, 0, 0],  "P",  [1, 0, 0]),
    ("mount    umr_base -> base_link", [0.0, 0.0, 0.42], [0.0, 0.0, 0.0], None, None),
] + [(name, xyz, rpy, "R" if axis is not None else None, axis) for name, xyz, rpy, axis in UR_CHAIN]
```

#### 1.2 DH table (vendor values, validated against the URDF chain):
```python
DH_UR = [                     
    (0.1807,  0.0,      pi/2),
    (0.0,    -0.6127,   0.0), 
    (0.0,    -0.57155,  0.0), 
    (0.17415, 0.0,      pi/2),   
    (0.11985, 0.0,     -pi/2),   
    (0.11655, 0.0,      0.0), 
]
T_BASE_UR = origin([0, 0, 0], [0, 0, pi])          # base_link -> DH frame 0 (yaw pi)
T_TOOL    = origin([0, 0, 0], [0, -pi/2, -pi/2])   # DH frame 6 -> flange

P6R_DH = [("P", 0.0,   0.0, 0.0, -pi/2)]           # row 0: slide along z_0 = world x
P6R_DH += [("R", pi/2, *DH_UR[0])]                 # row 1: UR row 1 with +pi/2 theta offset
P6R_DH += [("R", 0.0,  *row) for row in DH_UR[1:]] # rows 2..6: UR rows unchanged
T_BASE_P6R = np.eye(4)                             # world -> DH frame 0 at the mount height,
T_BASE_P6R[:3, :3] = [[0, 0, 1],                   # columns (x0, y0, z0) = (world y, world z, world x)
                      [1, 0, 0],
                      [0, 1, 0]]
T_BASE_P6R[:3, 3] = [0, 0, 0.42]
```

#### 1.3 Screw & PoE
```python
UR_AXES = [
    ("1 shoulder_pan",  [0, 0,  1], [0,       0,       0.1807 ]),
    ("2 shoulder_lift", [0, 1,  0], [0,       0,       0.1807 ]),
    ("3 elbow",         [0, 1,  0], [0.6127,  0,       0.1807 ]),
    ("4 wrist_1",       [0, 1,  0], [1.18425, 0.17415, 0.1807 ]),
    ("5 wrist_2",       [0, 0, -1], [1.18425, 0.17415, 0.06085]),
    ("6 wrist_3",       [0, 1,  0], [1.18425, 0.2907,  0.06085]),
]
P6R_SCREWS = [(np.zeros(3), np.array([1.0, 0, 0]))]                       
P6R_SCREWS += [screw(w, np.asarray(r, float) + mount) for _, w, r in UR_AXES] 
```

#### 1.4 Validating 3 models

The three models implement the following equations respectively (URDF chain, DH, PoE): 

$${}^{0}T_{n}(q) = {}^{0}T_{1}(q_1)\,{}^{1}T_{2}(q_2)\cdots{}^{n-1}T_{n}(q_n)$$

$${}^{i-1}T_{i} = R_{z}(\theta_i)\,T_{z}(d_i)\,T_{x}(a_i)\,R_{x}(\alpha_i)$$

$${}^{W}T_{E}(q) = e^{\widehat{{}^{W}\boldsymbol{\xi}_1}\,q_1}\cdots e^{\widehat{{}^{W}\boldsymbol{\xi}_n}\,q_n}\,M$$

Input: x = 0.5 m, q1..q6 = [10, -60, 90, -120, -90, 30] deg

1.4.1 T_world_flange, URDF chain

```text
-0.000000  -0.342020  -0.939693  1.376941
-0.000000  -0.939693   0.342020  0.331465
-1.000000   0.000000   0.000000  0.728989
 0.000000   0.000000   0.000000  1.000000
```

1.4.2 T_world_flange, DH table

```text
-0.000000  -0.342020  -0.939693  1.376941
 0.000000  -0.939693   0.342020  0.331465
-1.000000   0.000000   0.000000  0.728989
 0.000000   0.000000   0.000000  1.000000
```

1.4.3 T_world_flange, PoE

```text
-0.000000  -0.342020  -0.939693  1.376941
 0.000000  -0.939693   0.342020  0.331465
-1.000000  -0.000000   0.000000  0.728989
 0.000000   0.000000   0.000000  1.000000
```

1.4.4 Pairwise errors at the input q

```text
pair                    position [m]  orientation [deg]
URDF chain vs DH table      2.54e-16           1.14e-14
URDF chain vs PoE           2.78e-16           1.44e-14
DH table vs PoE             2.72e-16           3.17e-15
```

1.4.5 Worst errors over 1000 random q  (x in [-1.5, 1.5] m, angles in [-pi, pi])

```text
pair                    max position [m]  max orientation [deg]
URDF chain vs DH table          6.66e-16               3.03e-14
URDF chain vs PoE               1.28e-15               2.87e-14
DH table vs PoE                 1.22e-15               2.51e-14
```

Verdict: the three models agree (tolerance 1e-09 m, 1e-06 deg).

### 2. Jacobians
#### Symbolic Jacobians of the P6R: 0J6 (J_red) and the full 6 x 7 J

```text
1. 0J6: arm Jacobian in DH frame 0 of the arm
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
```
```text
2. WJ6 = J_red: the same arm Jacobian in the world frame

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
```
```text
3. WJ: whole-body Jacobian (6 x 7) in the world frame, columns (x, q1..q6) 
  WJ = [ e | WJ6 ],   e = [1 0 0 0 0 0]^T: the slide column is pure x translation and contains no q.
  J_red = WJ[:, 1:] = WJ6 (x fixed).

```



#### 2.2 Decoupling 3J5 of the UR12e: 1 + 3 + 2: plane split instead of wrist split (x fixed)
```text

Axes 2, 3, 4 are parallel, so joints q2, q3, q4 form a planar 3R inside the arm plane: they slide O5 within the plane and rotate about its normal, nothing else. Joint q1 turns the plane about the vertical, joints q5, q6 tilt the tool out of the plane and meet at O5. With the velocity point at O5 and the components expressed in frame 3 (x, y in the plane, z along its normal), reorder rows and columns:

   rows:     in-plane motion (vx, vy, wz) | out-of-plane translation (vz) | out-of-plane rotation (wx, wy)
   columns:  planar group (q2, q3, q4)    | shoulder (q1)                 | wrist (q5, q6)

                     q2 q3 q4   |   q1   |  q5  q6
   vx vy wz      [     P        |   *    |    *     ]
   vz            [     0        |  -rho  |    0     ]
   wz wy ->      [     0        |   *    |    W2    ]
   (wx wy)

   P  = [ a2 s3 + d5 c4       d5 c4       d5 c4 ]        planar 3R at O5
        [ a2 c3 + a3 + d5 s4  a3 + d5 s4  d5 s4 ]
        [ 1                   1           1     ]

   rho = a2 c2 + a3 c23 + d5 s234                        in-plane radial coordinate of O5

   W2 = [  s4   -s5 c4 ]                                 out-of-plane rotations from q5, q6
        [ -c4   -s5 s4 ]

The zeros: the parallel joints produce no vz, wx, wy; the wrist joints do not move O5.  The matrix is block
triangular, so

   det 3J5 = det P * (-rho) * det W2 = (a2 a3 sin q3) * (-rho) * (-sin q5)
           = a2 a3 sin q3 sin q5 (a2 c2 + a3 c23 + d5 s234)

Three blocks, three families, and the three groups of the analytical IK:

   block  size  joints      vanishes when                family     IK group
   P      3x3   q2 q3 q4    sin q3 = 0                   elbow      planar 3R, elbow up/down
   rho    1x1   q1          a2 c2 + a3 c23 + d5 s234 = 0 shoulder   q1 from O5, left/right
   W2     2x2   q5 q6       sin q5 = 0                   wrist      q5, q6 from orientation, flip
```
```text
Three families, one block each:
   - planar arm block P         sin q3 = 0     elbow family
   - shoulder entry rho         rho = 0        shoulder family: the only entry that moves O5 out of the
                                               plane direction vanishes (O5 above the shoulder line)
   - wrist 2x2 in Q             sin q5 = 0     wrist family: out-of-plane rotations from q5, q6 collapse
```
<img src="figures/elbow_s.png" width="32%">
<img src="figures/shoulder_s.png" width="32%">
<img src="figures/wrist_s.png" width="32%">


#### 2.3 Reduced J_red vs full J at representative configurations

For the UR12e there are 3 singularity families: shoulder, elbow, wrist. At a singular configuration the arm loses one task direction u (the left null vector of J_red). Every singular configuration falls into one of two types: 
- Type A: u has an x component, so the slide can supply the lost direction and the whole body keeps rank J = 6. 
- Type B: u has no x component, so the whole body is singular too, rank J = 5. 

Which type occurs depends on the configuration (in particular the base yaw q1), not on the family: the same family is type A at some yaws and type B at others.

Elbow Example

<img src="figures/elbow_singularity_1.png" height="250">

Elbow yaw 0 (type A):  x = 0.5 m,  q = [  0 -45   0 -45 -90   0] deg
```text
  sigma_min(J_red) = 3.72e-18     sigma_min(J) = 0.4472
  lost direction u = [ 0.6918  0.1259  0.6918 -0.0147  0.1635 -0.    ]  |u_x| = 0.6918

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 0.0000,   base speed xdot = 1.4455 m/s
          -> the slide supplies what the arm cannot

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q3)
           x - x0 [m]  pose error  held?  sigma_min(J_red)     q3 [deg]   q [deg]
               -0.300    7.67e-01     no            0.0636     -15.4446   [-162.6 -169.   -15.4  -85.6   90.    17.4]
               -0.250    2.84e-01     no            0.1873     -46.3608   [ -0.  -15.  -46.4 -28.6 -90.   -0. ]
               -0.200    3.56e-01     no            0.2584      68.8312   [ -0.  -72.1  68.8 -86.8 -90.   -0. ]
               -0.150    3.43e-01     no            0.2826     -73.1703   [ -0.   -5.  -73.2 -11.9 -90.   -0. ]
               -0.100    7.89e-02     no            0.0483     -11.6948   [ -0.  -36.  -11.7 -42.3 -90.   -0. ]
               -0.050    3.59e-02     no            0.0021       0.5076   [ -0.  -43.6   0.5 -46.9 -90.   -0. ]
                0.000    4.83e-07    yes            0.0004       0.1047   [ -0.  -45.1   0.1 -45.1 -90.   -0. ]
                0.050    3.29e-11    yes            0.1126      27.8705   [ -0.  -60.2  27.9 -57.7 -90.   -0. ]
                0.100    1.86e-11    yes            0.1560      39.1812   [ -0.  -67.5  39.2 -61.7 -90.   -0. ]
                0.150    4.25e-10    yes            0.1873      47.6671   [ -0.  -73.6  47.7 -64.1 -90.   -0. ]
                0.200    9.69e-11    yes            0.2119      54.6296   [ -0.  -79.   54.6 -65.6 -90.   -0. ]
                0.250    3.68e-11    yes            0.2320      60.5658   [ -0.  -84.1  60.6 -66.5 -90.   -0. ]
                0.300    1.83e-11    yes            0.2488      65.7245   [ -0.  -88.9  65.7 -66.8 -90.   -0. ]
          -> rescued for x - x0 in {+0.05 .. +0.30} m: pose held and the arm is regular
```
<img src="figures/elbow_singularity_2.png" height="250">

Elbow vertical (type B):   x = 0.5 m,  q = [  0 -90   0   0 -90   0] deg
```text
  sigma_min(J_red) = 7.82e-17     sigma_min(J) = 0.0000
  lost direction u = [ 0.      0.      0.9929  0.      0.119  -0.    ] |u_x| = 0.0000

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 1.0000,   base speed xdot = 0.0000 m/s
          -> the slide cannot help either

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q3)
           x - x0 [m]  pose error  held?  sigma_min(J_red)     q3 [deg]   q [deg]
               -0.300    7.24e-02     no            0.1135      27.9070   [  0.  -89.4  27.9 -28.5 -90.    0. ]
               -0.250    4.98e-02     no            0.0951     -22.9849   [  0.  -66.9 -23.   -0.1 -90.    0. ]
               -0.200    5.80e-02     no            0.1252     -30.3800   [ -0.1 -65.8 -30.4   6.1 -90.   -0.1]
               -0.150    1.03e-02     no            0.0182       4.3886   [ -0.  -84.9   4.4  -9.5 -90.   -0. ]
               -0.100    6.57e-03     no            0.0305      -7.3154   [ -0.1 -81.7  -7.3  -1.1 -90.   -0.1]
               -0.050    1.05e-03     no            0.0017      -0.4076   [ -0.  -87.4  -0.4  -2.2 -90.   -0. ]
                0.000    4.76e-07    yes            0.0004       0.1032   [  0.  -90.    0.1  -0.1 -90.    0. ]
                0.050    1.05e-03     no            0.0014      -0.3392   [  0.1 -92.2  -0.3   2.6 -90.    0.1]
                0.100    5.61e-03     no            0.0179      -6.5850   [  2.6 -91.3  -6.6   7.8 -90.    2.6]
                0.150    3.50e-03     no            0.0088      -2.3017   [ 20.3 -93.1  -2.3   5.4 -90.   20.3]
                0.200    6.59e-04     no            0.0000       0.0000   [ 49.5 -91.9   0.    1.9 -90.   49.5]
                0.250    4.45e-05     no            0.0000       0.0000   [ 73.5 -89.5   0.   -0.5 -90.   73.5]
                0.300    1.69e-03     no            0.0080       1.9263   [ 91.9 -88.    1.9  -3.9 -90.   91.9]
          -> no x holds the pose: reach boundary in a direction the slide cannot fix
```

### 3. Analytical IK solver

Megi

### 4. Reachable-position maps over discrete base positions
#### 4.1 Reachable-position maps, joint limits +/- pi

The maps show the geometric branch count: every joint angle is wrapped to (-pi, pi), so a configuration and its +/- 2 pi copie from UR12e are counted once. The UR12e's own limits (+/- 2 pi on all joints, +/- pi on the elbow) are never tighter than 180 deg, so they remove no geometric branch and the count needs no further filtering.  Any tighter limit would act as a filter on the same grid, lowering the count where a branch leaves the allowed interval.

![reach map x = 0](figures/reach_map_x0.png)
```text
flange orientation R (world), columns = flange axes:
[[ 0.  0.  1.]
 [ 0.  1.  0.]
 [-1.  0.  0.]]

flange height z = 0.9 m  (arm base at 0.42 m, shoulder axis at 0.6007 m)

grid: x_w in [-2.4, 2.4] m, y_w in [-1.4, 1.4] m, step 0.02 m, 241 x 141 = 33981 IK calls per map, 40 s for 3 maps
```
#### 4.2 Reachable-position maps with discrete base x
![reach maps](figures/reach_maps_x.png)

```text
base at x = -1.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m

base at x = +0.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m

base at x = +1.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m
```
reading: 
- the count is constant on regions bounded by the images of the three singular families:

    - Outer boundary: elbow family (arm stretched)---0 IK  
    - Inner hole of radius d4 = 0.174 m: shoulder family(O5 cannot come closer to axis 1 than the arm-plane offset)---0 IK  
    - Inside r = 1.0 m all 8 branches exist---8 IK
    - In the ring 1.0 .. 1.24 m only one wrist choice keeps O4 within the planar 2R reach---4 IK

- Moving the base translates the whole map: the slide adds reach only along x.

#### 4.3 Out of curiosity, Reachable-position maps with different flange orientation 
![reach maps](figures/reach_map_x0_tilt.png)

### 5. Cartesian path and the (s, x) redundancy plane

(the final chart will hit the a wall)

With x fixed the robot is a 6R arm with at most 8 IK branches. Letting x vary turns every branch into a one-parameter family of solutions. The redundancy is drawn as a
map with the path progress s on the horizontal axis and the redundant joint, here the base position x, on the vertical axis. Every dot is one analytical IK evaluation.

#### 5.1 Path

A closed rectangle in the horizontal plane, tool axis pointing down, longer than the arm can reach from one base position, so the base has to move:

```text
corners (x_w, y_w)   (-0.8, 0.25) -> (0.8, 0.25) -> (0.8, 0.85) -> (-0.8, 0.85) -> back     
flange height        z = 0.9 m
orientation          flange outward normal (axis 6) pointing down, flange z along +x_w
s                    arc length over the 4.4 m perimeter, 111 samples
x                    base position, -1.0 .. 1.0 m, step 0.05 m (41 positions)
```

#### 5.2 Classification of one dot

For each (s, x) the analytical IK of the arm is run with the base at x. A branch gets a green dot when it has a root and the fixed-x arm is regular, sigma_min(J_red) >= 0.03; otherwise grey (no root, or the root is singular). Joint limits are +/- pi, so no root is lost to limits. On this path the singular classes stay empty: the tool-down orientation keeps q5 at +/- 90 deg, far from the wrist family, and the rectangle stays
outside the shoulder hole of radius d4. The only family touched is the elbow, at the edge of the reach band, where the root disappears at the same place.

#### 5.3 Map

<img src="figures/sx_map.png" width="100%">

One panel per branch. The green band follows the x coordinate of the tool along the rectangle: it rises on the first long side and falls on the return. No horizontal line stays green in any panel, so a parked base cannot execute this path.


#### 5.4 Base schedule x(s), analytical planning

Within one branch the green dots form a layered graph over s. Two dots in neighbouring layers are connected when the base moves at most 0.05 m and every joint moves at most 0.5 rad, measured on the unwrapped angles so that a jump across +/- pi is rejected. Dynamic programming returns the cheapest complete schedule from the fixed start x(0) = -1.0 m with

```text
cost = sum over steps of ( |dq|_2 [rad]  +  2 * |dx| [m] )
```

i.e. joint travel plus a penalty on base travel; it is a distance, not a time.

```text
branch     plan cost   x range of the schedule
S+ W+ E+      11.79    [-1.00, +0.40] m
S+ W+ E-       none    q4 crosses +/- pi along the path
S+ W- E+      10.31    [-1.00, +0.05] m      <- best
S+ W- E-      10.51    [-1.00, +0.05] m
S- W+ E+       none    q2 and q4 cross +/- pi along the path
S- W+ E-      11.04    [-1.00, +0.10] m
S- W- E+      12.97    [-1.00, +0.50] m
S- W- E-      12.76    [-1.00, +0.45] m

best branch S+ W- E+: base from x = -1.00 to -0.15 m, base travel 1.25 m,
joint travel per joint [deg] = [162, 214, 247, 81, 0, 162],
min sigma_min(J_red) along the schedule = 0.172
```

The red line in the figure is the schedule of the best branch (thin red: the other feasible branches). The
base climbs with the tool during the first long side, waits near x = 0 during the short side and the start
of the return, then steps back to -0.15 m.

#### 5.5 Findings

- The base position decides which branches exist at a given path point (the width and position of the
  green band differ per branch) and whether a continuous execution is possible at all: six of the eight
  branches admit a complete schedule, none admits a parked base.
- Two branches, S+ W+ E- and S- W+ E+, are mostly green yet have no schedule. Along the path one of their
  joints (q4, and q2 for the second) runs through 180 deg. Each dot is inside +/- pi, so the map shows no
  defect, but a path cannot cross the +/- pi seam. The joint-limit convention acts as a path constraint, not
  as a filter on the dots.
- The planner is fully analytical: every configuration on the red line is a closed-form IK root, the
  Jacobian enters only through the sigma_min check. A greedy planner that keeps the base at x = -1.0 m
  loses the pose at s = 0.23, when the tool leaves the arm's reach.