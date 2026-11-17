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

Three factors, sin q3 (elbow, E), rho = a2 c2 + a3 c23 + d5 s234 (shoulder, S) and sin q5 (wrist, W): three blocks, three families, and the three groups of the analytical IK:

   block  size  joints      vanishes when                family     IK group
   P      3x3   q2 q3 q4    sin q3 = 0                   elbow      planar 3R, elbow up/down
   rho    1x1   q1          a2 c2 + a3 c23 + d5 s234 = 0 shoulder   q1 from O5, left/right
   W2     2x2   q5 q6       sin q5 = 0                   wrist      q5, q6 from orientation, flip
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

Solver: `scripts/ik_ur12e.py`, validation: `scripts/step05_ik.py` (output in `results/step05_ik.md`).

#### 3.1 Problem: fixed x turns the P6R into a UR12e with a shifted target

With the base at x, the world flange target T* is moved into DH frame 0 of the arm:

$${}^{0}T_{6} = \big(\mathrm{Trans}_x(x)\; T_{\mathrm{mount}}\; T_{\mathrm{BASE}}\big)^{-1}\; T^{*}\; T_{\mathrm{TOOL}}^{-1}$$

From here on the problem is a pure 6R problem, and x no longer appears in the formulas: changing x only changes the
target that the arm sees.

#### 3.2 Plane split: three groups instead of a wrist split

The UR wrist is offset (d5, d6 ≠ 0), so axes 4, 5, 6 do not meet in one point and Pieper's position/orientation
split does not apply. Section 2.2 gives the decoupling that does apply. Axes 2, 3, 4 are parallel, so 3J5 is block
triangular, and its three blocks are the three groups of the IK:

```text
group  joints      quantity used                         2 roots    singular when (block of 2.2)
1      q1          position of O5 (wrist_2 origin)       S+ / S-    rho = 0        shoulder
2      q5, q6      orientation, given q1                 W+ / W-    sin q5 = 0     wrist
3      q2, q3, q4  planar 3R in the arm plane            E+ / E-    sin q3 = 0     elbow
```

Each group uses a quantity that depends only on the joints still unknown in that group. The groups are solved in
the order 1 -> 2 -> 3.

#### 3.3 Group 1: q1 from O5 (shoulder S±)

O5 is where the wrist_2 and wrist_3 axes cross, d6 behind the flange on the tool axis:
$p_5 = p - d_6 z_6$. It is fixed by the target alone. The upper arm and forearm move in a vertical plane through
axis 1. The wrist sits d4 to the side of that plane, and q2..q5 cannot change this offset. So the plane, turned by
q1, must leave O5 at distance d4. Seen from above, the plane is a tangent from axis 1 to a circle of radius d4
around O5, and there are two tangents:

$$q_1 = \mathrm{atan2}(p_{5y},\, p_{5x}) \pm \arccos\!\left(\frac{d_4}{r}\right) + \frac{\pi}{2},
\qquad r = \sqrt{p_{5x}^2 + p_{5y}^2} \;\ge\; d_4$$

- r < d4: no root (the inner hole of the maps in section 4).
- r = d4: the two tangents coincide (shoulder family, rho = 0).

Example (pose of 1.4): O5 = (-0.877, -0.332, 0.426) m in frame 0, r = 0.9375 m.
atan2 = -159.3°, arccos(d4/r) = 79.3°, so q1 = **10.0°** (S+) or **-148.6°** (S-).

#### 3.4 Group 2: q5, q6 from the orientation (wrist W±)

With q1 known, the common direction of axes 2-3-4 is $n = (\sin q_1, -\cos q_1, 0)$ in frame 0. Along n, O5 sits
at d4 (3.3), and the flange is O5 + d6 z6, so the tool axis has component $\cos q_5$ along n:

$$\cos q_5 = \frac{p \cdot n - d_4}{d_6}, \qquad q_5 = \pm \arccos(\cdot)$$

The sign of q5 is the wrist flip. q6 follows from the components of x6 and y6 along n:

$$q_6 = \mathrm{atan2}\!\big(-\mathrm{sgn}(\sin q_5)\,(y_6 \cdot n),\;\; \mathrm{sgn}(\sin q_5)\,(x_6 \cdot n)\big)$$

because, seen from the tool frame, n has the components $(\sin q_5 \cos q_6,\, -\sin q_5 \sin q_6,\, \cos q_5)$.

At sin q5 = 0 (wrist family) the tool axis is parallel to axes 2-3-4. Then q4 and q6 turn about the same axis,
only q4 + q6 is determined, and the solver sets q6 = 0.

#### 3.5 Group 3: q2, q3, q4 as a planar 3R (elbow E±)

With q1, q5, q6 known, the three outer transforms are peeled off:

$${}^{1}T_{4} = A_1^{-1}\; {}^{0}T_{6}\; A_6^{-1} A_5^{-1} = A_2 A_3 A_4$$

The point $p_{13}$ (O4 without its d4 offset, in frame 1) lies in the arm plane and is reached by the two links
a2, a3. The law of cosines gives

$$\cos q_3 = \frac{|p_{13}|^2 - a_2^2 - a_3^2}{2 a_2 a_3}, \qquad q_3 = \pm\arccos(\cdot), \qquad
q_2 = -\mathrm{atan2}(p_{13,y}, -p_{13,x}) + \arcsin\!\frac{a_3 \sin q_3}{|p_{13}|}$$

and q4 is the remaining in-plane rotation, read off ${}^{3}T_{4}$ (q2 + q3 + q4 = in-plane angle of the wrist).

- |cos q3| > 1: no root (outer reach of the maps).
- cos q3 = ±1: arm stretched or folded (elbow family).

#### 3.6 Branches, labels, angle wrapping

- Three groups with two roots each give at most 2 × 2 × 2 = **8 branches**. A branch is dropped when an acos
  argument leaves [-1, 1].
- Every angle is wrapped to (-π, π]. With the project limits of ±π on every joint, each geometric branch gives
  exactly one joint vector and no root is rejected by a limit. On a path, however, the ±π seam acts as a
  joint limit (section 5).
- **Label = sign triple** (sign rho, sign q5, sign q3). Each branch lies in its own aspect, the region cut out by
  the three singular surfaces det J_red = 0. `branch_of(q)` reads the label off any joint vector.

**Why shoulder / wrist / elbow names, when the UR has no shoulder–wrist decoupling?** What fails on the UR is the
classic split into an arm that positions a wrist centre (q1–q3) and a spherical wrist that orients the tool
(q4–q6). The names are not used in that sense here. The IK still has exactly three binary choices, and each is made
at one joint: shoulder pan q1, elbow q3, wrist 2 q5 (the UR's own joint names). Each label is *defined* as the sign
of one factor of det J_red = a2 a3 sin q3 sin q5 rho, so the labels are the aspects of the robot.
- **S** (sign rho): whether O5 lies in front of or behind the shoulder axis. This is the same meaning as
  "shoulder left/right" on an anthropomorphic arm.
- **E** (sign sin q3): the bend of the planar 2R. Its physical "up/down" flips with the shoulder side, which is
  why signs are used rather than up/down.
- **W** (sign sin q5): computed from the flange *position* (cos q5 = (p·n − d4)/d6), not from the orientation
  alone. This coupling is exactly what a spherical wrist would remove.

The UR has one solution per aspect: the sign triple read from q matches the label in all 14 426 cases of 3.7. The
labels are therefore unambiguous and change only across a singular surface. They could equally be written
(σ_rho, σ_5, σ_3); the names only say where each choice is made.

**IK and det J_red are the same three conditions.** Each group of 3.2 has exactly one arccos, and its two roots
merge exactly where one factor of det J_red vanishes:

```text
group  root formula                 roots merge when      factor of det J_red = 0
1      q1 = ... ± arccos(d4 / r)    r = d4                rho = 0      (shoulder)
2      q5 = ± arccos(...)           cos q5 = ±1           sin q5 = 0   (wrist)
3      q3 = ± arccos(...)           cos q3 = ±1           sin q3 = 0   (elbow, det P = 0)
```

So branches are created and destroyed in pairs on det J_red = 0, which is why every branch count in 3.7, 3.9
and section 4 is even.

**The UR12e is not cuspidal.** A cuspidal robot can move from one IK solution to another without crossing a
singularity, i.e. it has more than one solution in the same aspect. Here the three sign factors of det J_red
split joint space into 8 aspects and each aspect holds exactly one solution (0 mismatches in 14 426 cases, 3.7).
A solution can therefore only change its label by crossing det J_red = 0, which is what the persistence check of
3.7 shows: all label switches happen next to a singular surface. For path planning this means a branch label
fixed at the start stays valid until the path meets a singularity.

#### 3.7 Validation

From `results/step05_ik.md` (2000 random configurations, limits ±π):

```text
max |q_true - nearest branch|  = 6.4e-11 rad   (the input configuration is always recovered)
max |FK(branch) - T06|         = 3.4e-12       (every returned branch reaches the pose)
number of branches per pose    = {2: 51, 4: 268, 6: 98, 8: 1583}
label vs sign triple read from q: 14426 solutions, 0 mismatches
```

The count is always even, because roots appear and disappear in pairs at the singular surfaces.

**Label persistence along paths** (`drafts/ik_persistence_check.py`). We took 100 random straight joint-space
paths with 300 steps each and ran FK then IK at every step. For each label we checked whether the nearest root
at the next step carries the same label:

```text
same label in 99.72 % of 194256 steps;  the 545 switches, by closest singular family:
  shoulder (rho = 0) 68.4 %    wrist (sin q5 = 0) 23.9 %    elbow (sin q3 = 0) 7.7 %
```

**Vectorised version.** `scripts/ik_ur12e.py` solves one pose per call, which is enough for the maps of
section 4 (about 34 000 calls per map). `drafts/ik_ur12e_full.py` implements the same three groups vectorised over
arrays of poses (all 8 sign combinations at once, no loop over poses). It runs at about 59 000 poses/s with 8
branches each, recovers the original configuration on 200 000 random samples (worst error 2.4e-10 rad), and gives
the same 8 solutions as the table in 3.8.

Labels change only next to a singular surface, where two branches meet. A path planner therefore follows a
branch by continuity and monitors sigma_min; the label alone is not enough (section 5).

**Independent check in the course simulator (Module 3).** The 8 solutions of the pose of 1.4 were loaded into
Module 3, which computes forward kinematics with its own code. All 8 show 0.000 mm / 0.0000°. Files:
`drafts/sim/umr_ur12e_sim_report.urdf` and `drafts/sim/m3_section3_pose.csv`. The repo's
`sim/umr_ur12e_sim.urdf` mounts the arm 1 cm lower and rotated by π about axis 1, so it does not match this
report's model; the corrected copy matches the URDF chain to 6e-10.

<img src="figures/module3_ik_solutions.png" width="100%">

Module 3 with `drafts/sim/umr_ur12e_sim_report.urdf`, end-effector link `tool0`, target typed as
(1.3769, 0.3315, 0.729) m, roll/pitch/yaw = (180, 0, 110)°. Top: all 8 ghosts and the error table; bottom: each
solution alone, numbered as the rows of `m3_section3_pose.csv` (same order as the table in 3.8). The common
0.055 mm is the rounding of the typed target to 4 decimals (exact target (1.376941, 0.331465, 0.728989) m: the
difference is 0.055 mm). It is the same for every row, so it is not an IK error; the orientation error is 0.0000°.
**Collisions are not part of the IK.** Approximating the arm by the polyline through its joint origins, the lowest
point above the UMR deck footprint (top at z = 0.41 m) is: solutions 1, 3, 6, 8 +19 cm; 2 +11 cm; 7 +10 cm;
4 -1 cm; 5 -4 cm. With link radii of about 6-7.5 cm, solutions **4 and 5 pass through the base**, and 2 and 7 clear
it by only a few centimetres. All 8 are valid IK solutions, but only about half are physically usable from this
base position. A collision check (not part of this project) would remove 4 and 5.

#### 3.8 All solutions of one tool pose

Target = flange pose of 1.4 (x = 0.5 m, q0 = [10, -60, 90, -120, -90, 30] deg):

```text
branch           q1       q2       q3       q4       q5       q6       rho  sin q3  sin q5   det J_red    |FK-T|
S+ W+ E+      10.00   -37.74    50.91    76.83    90.00  -150.00    -0.921   0.776   1.000  -2.504e-01   1.5e-15
S+ W+ E-      10.00    11.27   -50.91   129.64    90.00  -150.00    -0.921  -0.776   1.000   2.504e-01   1.9e-15
S+ W- E+      10.00   -60.00    90.00  -120.00   -90.00    30.00    -0.921   1.000  -1.000   3.226e-01   1.5e-15   <- q0
S+ W- E-      10.00    26.02   -90.00   -26.02   -90.00    30.00    -0.921  -1.000  -1.000  -3.226e-01   1.5e-15
S- W+ E+    -148.59   153.98    90.00  -153.98    90.00    51.41     0.921   1.000   1.000   3.226e-01   1.3e-15
S- W+ E-    -148.59  -120.00   -90.00   -60.00    90.00    51.41     0.921  -1.000   1.000  -3.226e-01   1.3e-15
S- W- E+    -148.59   168.73    50.91    50.36   -90.00  -128.59     0.921   0.776  -1.000  -2.504e-01   1.3e-15
S- W- E-    -148.59  -142.26   -50.91   103.17   -90.00  -128.59     0.921  -0.776  -1.000   2.504e-01   1.3e-15
```

<img src="figures/ik_all_solutions.png" width="100%">

The same tool pose is reached by 8 physically different postures. The two shoulder sides differ by about 159° in
q1, and within each side the wrist flips and the elbow goes up or down.

#### 3.9 The base position selects the branches

Same world target, base slid along x (`results/step05_ik.md` section 5, and a finer sweep):

<img src="figures/ik_branches_vs_x.png" width="80%">

```text
x <  0.13 m          0 branches  (target out of reach)
0.13 <= x < 0.38 m   4 branches  S+ W- E±, S- W+ E±
x >= 0.38 m          8 branches
```

Branches come and go in elbow pairs. Fixing x does not just shift the arm; it decides which branches exist.
Sections 4 and 5 extend this from one pose to a grid of positions and to a path.

### 4. Reachable-position maps over discrete base positions

Scripts: `scripts/step06_reach_map.py` (branch-count maps, 4.1–4.3), `drafts/maps_section4.py` (per-branch
classes, overlay and singular cells, 4.1, 4.2 and 4.4; output `drafts/maps_section4_output_z09.txt`).

#### 4.1 Branch-count map with the base fixed

For every grid point the flange is placed there with a fixed orientation, and the analytical IK of section 3
is run once. The colour is the number of branches that return a root.

```text
flange orientation R (world), columns = flange axes:
[[ 0.  0.  1.]
 [ 0.  1.  0.]
 [-1.  0.  0.]]          -> tool axis (flange x = DH z6) pointing down

flange height z = 0.9 m  (arm base at 0.42 m, shoulder axis at 0.6007 m)
grid: x_w in [-2.4, 2.4] m, y_w in [-1.4, 1.4] m, step 0.02 m, 241 x 141 = 33981 IK calls per map
```

The count is the geometric branch count. Every joint angle is wrapped to (-pi, pi], so a configuration and its
+/- 2 pi copies on the real UR12e are counted once. The UR12e limits (+/- 2 pi, elbow +/- pi) are never tighter
than 180 deg, so they remove no branch here. A tighter limit would act as a filter on the same grid.

![reach map x = 0](figures/reach_map_x0.png)

The count is constant on regions whose borders are the singular families of 2.2, and it changes in pairs:

- **Inner hole, radius d4 = 0.174 m (0 IK): shoulder family.** O5 cannot come closer to axis 1 than the
  arm-plane offset d4, so q1 has no root (3.3).
- **Outer edge, r = 1.24 m (0 IK): elbow family.** The planar 2R cannot stretch further, |cos q3| > 1 (3.5).
- **Disc r < 1.0 m (8 IK)** and **ring 1.0 .. 1.24 m (4 IK).** With the tool vertical, q5 = +/- 90 deg, so the
  wrist link d5 points either away from axis 1 or towards it. Only the outward wrist choice reaches the ring,
  and the two reach radii differ by about 2 d5 = 0.24 m. The 4 surviving branches are S+ W- E± and S- W+ E±.
- **No wrist family.** The tool axis is vertical and the arm-plane normal n is horizontal, so
  cos q5 = 0 everywhere (3.4); sin q5 = 0 cannot occur with the tool pointing down.

The same map split by branch shows the two reach classes directly (green = root with sigma_min(J_red) >= 0.05):

<img src="figures/map_down_branches_x0.png" width="100%">

#### 4.2 Moving the base

![reach maps](figures/reach_maps_x.png)

```text
base at x = -1.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m
base at x = +0.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m
base at x = +1.0 m: 0 IK: 22129 pts, 4 IK: 4176 pts, 8 IK: 7676 pts;  reachable radius from axis 1: 0.179 .. 1.241 m
map at x = +1 / -1 equals the map at x = 0 shifted by +1 / -1 m: True
```

Moving the base translates the whole map; it does not change its shape. The slide therefore adds reach along x
only. What changes for a fixed world target is *which* region of the map it falls in, i.e. how many branches
it has (3.9).

To see what this means for a task, the maps for x = -1, 0, +1 m are overlaid on a task strip
x_w in [-1.5, 1.5] m, y_w in [0.5, 0.9] m:

<img src="figures/map_down_overlay.png" width="80%">

```text
strip covered by >= 1 feasible branch     tool down
base at x = -1 m                             50.8 %
base at x =  0 m                             67.5 %
base at x = +1 m                             50.1 %
union of the three                          100.0 %
covered by one x only                        34.3 %
```

No single base position covers the strip. The three together do, but a third of the strip is covered by one x
only: there the base position is forced. Along a path this means the base has to move (section 5). The strip is a
generic 3 m task region, wider than the 1.6 m rectangle of section 5. The rectangle itself fits inside the reach
of a base at x = 0 on the far-reaching branches, which is why a parked base at x = 0 works there (5.4), while the
start position x = -1 m does not.

#### 4.3 Flange orientation

![reach maps tilted](figures/reach_map_x0_tilt.png)

Tilting the tool axis away from vertical (here towards +x, base at x = 0) breaks the rotational symmetry of the
map. The 4-branch ring narrows, and with the tool horizontal (90 deg) it has almost disappeared: nearly every
reachable point has 8 branches. The two reach classes of 4.1 exist because the wrist offset d5 lies in the
horizontal plane when the tool points down; once the tool tilts, that no longer holds.

The singular curves move with the orientation. With a horizontal tool the wrist family appears: thin wedges
where the tool axis is parallel to axes 2-3-4 (sin q5 = 0). For the tool along +Y the singular area is about
four times that of the tool-down case (0.26 against 0.07 m^2 per branch at x = 0):

<img src="figures/map_horizontal_branches_x0.png" width="100%">

With the tool down only two of the three singular families (shoulder, elbow) can appear; the horizontal tool is
needed to see the wrist family, which is the one the slide cannot rescue (4.4).

#### 4.4 Which singular cells the slide rescues (J_red vs J)

Each singular cell (sigma_min(J_red) < 0.05) is assigned to the det J_red factor of 2.2 closest to zero, and we
check whether the whole body is regular there (sigma_min(J) >= 0.05):

```text
                tool down                      tool horizontal (+Y)
shoulder     416 cells, rescued 93.3 %       438 cells, rescued 90.0 %
elbow        960 cells, rescued 88.3 %       992 cells, rescued 95.6 %
wrist          0 cells                      3960 cells, rescued 12.5 %
```

<img src="figures/map_down_sigma_2.png" width="80%">

This is the map-level version of the type A / type B result of 2.3:
- At the **elbow** boundary the arm loses a radial translation. The slide supplies it along +/- X (bright lobes)
  but not along +/- Y, where the radial direction is perpendicular to the slide.
- The **wrist** family mostly loses a rotation, which a prismatic joint cannot supply: only 12.5 % rescued.

The singular class stays a thin band around the critical curves for any threshold: 0.0 / 0.3 / 1.8 / 6.9 % of
the reachable cells for 0.01 / 0.02 / 0.05 / 0.10.

### 5. Cartesian path and the (s, x) redundancy plane

Scripts: `scripts/step07_sx_map.py` (map and DP), `scripts/step07b_sx_parked.py` (parked base),
`scripts/step07c_sx_tilted.py` (tilted path), `drafts/section5_planning.py` (fixed-x baseline, greedy, profiles).

With x fixed the robot is a 6R arm with at most 8 IK branches. Letting x vary turns every branch into a
one-parameter family of solutions. The redundancy is drawn as a map with the path progress s on the horizontal
axis and the redundant joint, here the base position x, on the vertical axis. Every dot is one analytical IK
evaluation.

#### 5.1 Path

A closed rectangle in the horizontal plane, tool axis pointing down, 1.6 m long. A base parked near x = 0
reaches all of it on the far-reaching branches only, and from the start position x = -1.0 m it cannot be
executed without moving the base:

<p align="center">
  <img src="figures/flatpath.png" width="50%">
</p>

```text
corners (x_w, y_w)   (-0.8, 0.25) -> (0.8, 0.25) -> (0.8, 0.85) -> (-0.8, 0.85) -> back
flange height        z = 0.9 m
orientation          flange outward normal (axis 6) pointing down, flange z along +x_w
s                    arc length over the 4.4 m perimeter, 111 samples
x                    base position, -1.0 .. 1.0 m, step 0.05 m (41 positions)
```

#### 5.2 Classification of one dot

For each (s, x) the analytical IK is run with the base at x. A branch gets a green dot when it has a root and
the fixed-x arm is regular, sigma_min(J_red) >= 0.03 (section 4 uses 0.05; the best schedules of 5.5 and 5.7 stay
at 0.172 and 0.145, well above either value); otherwise grey (no root, or a singular root). Joint limits
are +/- pi, so no root is lost to limits. On this path the singular classes stay empty: tool-down keeps q5 at
+/- 90 deg, away from the wrist family, and the rectangle stays outside the shoulder hole. The only family
touched is the elbow, at the edge of the reach band, where the root disappears at the same place.

#### 5.3 Map

<img src="figures/sx_map.png" width="100%">

One panel per branch. The green band follows the x coordinate of the tool along the rectangle: it rises on the
first long side and falls on the return. The branches come in the two reach classes of 4.1: the four with the
outward wrist (S+ W- E±, S- W+ E±) have the wider band.

#### 5.4 Parked base

A parked base is a horizontal line in the map. The line must stay green over the whole path, and the joints
must stay continuous between neighbouring dots.

<p align="center">
  <img src="figures/sx_map_parked.png" width="50%">
</p>

Only the rows x = -0.10 .. +0.10 m stay green over the whole path, and only on the four wide-band branches.
On S- W+ E+ a joint still has to cross the +/- pi seam, so three branches complete with a parked base:
S+ W- E+, S+ W- E-, S- W+ E-. From x = 0 the farthest corner is 1.167 m away, inside the 1.241 m reach of the
outward wrist, and the nearest path point is 0.25 m away, outside the shoulder hole.

Following each root from s = 0 with the base parked (|dq| <= 0.5 rad per step on the wrapped angles, so a jump
across +/- pi counts as reaching the joint limit):

```text
branch     | base parked at x = 0 m                          | base parked at x = -1 m
S+ W+ E+   |  44 % done, s = 0.45: no root (out of reach)    |  17 % done, s = 0.18: no root (out of reach)
S+ W+ E-   |   8 % done, s = 0.09: q4 reaches +-pi seam      |   5 % done, s = 0.05: q4 reaches +-pi seam
S+ W- E+   | completes                                       |  23 % done, s = 0.24: no root (out of reach)
S+ W- E-   | completes                                       |  23 % done, s = 0.24: no root (out of reach)
S- W+ E+   |   2 % done, s = 0.03: q4 reaches +-pi seam      |   2 % done, s = 0.03: q2 reaches +-pi seam
S- W+ E-   | completes                                       |  23 % done, s = 0.24: no root (out of reach)
S- W- E+   |  40 % done, s = 0.41: q2 reaches +-pi seam      |  15 % done, s = 0.15: q2 reaches +-pi seam
S- W- E-   |  44 % done, s = 0.45: no root (out of reach)    |  17 % done, s = 0.18: no root (out of reach)
```

<img src="figures/s5_fixed_x_baseline.png" width="100%">

There are only two failure causes on this path: the tool leaves the reach band (elbow family), or a joint
reaches the +/- pi seam. No branch stops on a singularity. Since no branches merge, the nearest root at the next
step always carries the same label (100 % of 1164 steps): the analytical labels are also the continuation labels.

#### 5.5 Moving base: branch-aware schedule x(s)

Within one branch the green dots form a layered graph over s. Two dots in neighbouring layers are connected when
the base moves at most 0.05 m and every joint moves at most 0.5 rad, measured on the unwrapped angles so that a
jump across +/- pi is rejected. Dynamic programming returns the cheapest complete schedule from the fixed start
x(0) = -1.0 m with

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

The red line in the map of 5.3 is the schedule of the best branch (thin red: the other feasible branches). The
base climbs with the tool during the first long side, waits near x = 0 during the short side and the start of
the return, then steps back to -0.15 m. Compared with the base parked at x = 0 on the same branch, joint travel
drops from 11.7 to 7.8 rad and the minimum sigma_min(J_red) rises from 0.115 to 0.172.

#### 5.6 Greedy continuation vs branch-aware planning

**Hypothesis.** Looking one step ahead is enough when the base is fast, but not when it is slow: then the base
has to commit early to obstacles that a local rule cannot see.

The proposal's greedy planner chooses x **locally**. At every step it takes one admissible neighbour (same
branch, |dx| <= 0.05 m, |dq| <= 0.5 rad, the same constraints as the DP) and never looks ahead. The greedy rule
is **min step cost**: the neighbour with the smallest |dq| + 2|dx|. This is exactly the DP cost, minimised one
step at a time, so the only difference between the two planners is the lookahead.

As a check that the result does not depend on the choice of local rule, we also ran a second one, **max
sigma_min** (the neighbour with the largest sigma_min(J_red), the safest-looking step). All runs start at
x(0) = -1 m. The DP of 5.5 is reproduced exactly (costs agree within 0.005).

```text
branch     | branch-aware DP | greedy, min step cost | greedy, max sigma_min
S+ W+ E+   | cost 11.79      | completes, cost 13.03 | completes, cost 15.31
S+ W+ E-   | no schedule     | fails s=0.42, q4 seam | fails s=0.42, q4 seam
S+ W- E+   | cost 10.31      | completes, cost 15.02 | completes, cost 14.99
S+ W- E-   | cost 10.51      | completes, cost 15.00 | completes, cost 14.42
S- W+ E+   | no schedule     | fails s=0.39, q2 seam | fails s=0.39, q2 seam
S- W+ E-   | cost 11.04      | completes, cost 13.95 | completes, cost 15.70
S- W- E+   | cost 12.97      | completes, cost 14.56 | completes, cost 15.04
S- W- E-   | cost 12.76      | completes, cost 14.98 | completes, cost 16.08
```

<img src="figures/s5_sx_map_dp_vs_greedy.png" width="100%">

With a fast base (|dx/ds| <= 1.25) greedy completes the same 6 branches as the DP, but costs 10–46 % more.
Minimising the cost of each step does not minimise the cost of the path: depending on the branch, greedy either
moves the base too little or too much. On the best branch (S+ W- E+) it moves the base too little and rides the
reach boundary (elbow family): sigma_min(J_red) drops to **0.033**, just above the threshold, against 0.172 for
the DP. On S- W+ E- and S- W- E± it moves the base up to x ≈ 0.95 m early, because there moving the base saves
more joint motion than it costs in that step. The safety rule (max sigma_min) also completes, but moves the base
3.9 m against 1.25 m for the DP.

**Slower base.** The slope bound is the base speed relative to the tool speed. Tightening it separates the planners:

```text
base speed <= k x tool speed     1.25   0.94   0.62   0.50   0.42   0.31
branch-aware DP                   6/8    6/8    5/8    3/8    3/8    0/8
greedy, min step cost             6/8    6/8    3/8    3/8    0/8    0/8
greedy, max sigma_min             6/8    6/8    3/8    2/8    0/8    0/8
```
(complete executions out of 8 branches; the x grid step equals the base step in each column)

<img src="figures/s5_sx_map_dp_vs_greedy_slow.png" width="100%">

At k = 0.42 the DP still finds three schedules and greedy fails on every branch, with either local rule, each time because
the green band moved out of reach. On the first long side the band climbs with the tool. A slow base keeps up only
if it starts climbing immediately and stays in the upper part of the band, so that it passes above the tip of the
grey wedge at s ≈ 0.5. Greedy (min step cost) hesitates one step and meets the tip of the wedge at s = 0.49; the
safety rule first waits where the arm is best conditioned and is too late by s = 0.33. No local rule can see the
wedge coming; the DP sees the whole (s, x) plane and commits early. The hypothesis holds: the failure comes from the
missing lookahead, not from the choice of local rule.

**Execution profiles** (best DP branch S+ W- E+ and its two greedy runs, fast base):

<img src="figures/s5_profiles.png" width="80%">

```text
run                    s reached  base travel  min seam margin  max pos residual  min sig(J_red)  min sig(J)
branch-aware DP             1.00      1.25 m         5.7 deg          1.1e-15           0.172        0.173
greedy, min step cost       1.00      2.00 m         4.3 deg          1.1e-15           0.033        0.173
greedy, max sigma_min       1.00      3.90 m         7.0 deg          9.2e-16           0.172        0.173
```

- The Cartesian residual is ~1e-15 m: every executed configuration is a closed-form IK root, re-checked through
  the independent URDF chain. Only the motion between samples is unchecked.
- On the first long side q6 sits within 4–7 deg of -180 deg on every run, close to the seam that removed
  S+ W+ E- and S- W+ E+. With the real UR limits of +/- 2 pi that seam disappears (not checked here).
- sigma_min(J) stays >= 0.17 on every run, even where the greedy-cost run takes sigma_min(J_red) down to 0.033:
  the arm alone is near the elbow singularity, the whole body is not, because the slide supplies the lost radial
  direction (type A, 2.3).

#### 5.7 Tilted path

<p align="center">
  <img src="figures/tilted20x30path.png" width="50%">
</p>
<img src="figures/sx_map_tilt_x30y20.png" width="100%">

The same rectangle is placed on a plane tilted 30 deg about world x and 20 deg about world y, with the tool axis
kept normal to the plane, (0.342, 0.470, -0.814) in the world; the flange height now varies from 0.66 to 1.44 m.

- The maps are no longer symmetric: the outbound and return sides differ in height and in distance from the
  rail, so every band is lopsided.
- A diagonal grey slit appears on the outbound side in all panels. It is the shoulder hole: with the tool
  leaning, O5 comes within d4 = 0.174 m of axis 1 when the base passes under that part of the path.
- Orange dots are roots at a singular configuration (sigma_min(J_red) < 0.03): 112 on the band edges (elbow
  family) and 73 along the slit (shoulder family), 8 of the latter singular for the whole body. q5 stays between
  54 and 126 deg, so the wrist family is not met.

```text
branch     plan cost   x range of plan      parked x that work   why no schedule from x = -1 m
S+ W+ E+       none    -                    none                 region not connected to the start (slit)
S+ W+ E-       none    -                    none                 q4 crosses +/- pi
S+ W- E+       none    -                    0.00 m               q4, q6 cross +/- pi
S+ W- E-      11.75    [-1.00, +0.10] m     0.00 m               <- best
S- W+ E+       none    -                    none                 q2, q4 cross +/- pi
S- W+ E-      11.92    [-1.00, +0.15] m     0.00 m
S- W- E+       none    -                    none                 q2, q6 cross +/- pi
S- W- E-      15.42    [-1.00, +0.70] m     none
```

From x = -1.0 m only three branches keep a complete schedule, and a parked base works at x = 0 only.

#### 5.8 Findings

- **The base position decides which branches exist along the path, and whether the path can be executed at all.**
  On the flat path six of eight branches admit a complete schedule from x = -1.0 m. A base parked at
  x = -1 m loses the pose by s = 0.24 on every branch; a parked base works only for x within +/- 0.10 m, on
  three branches. Even where parking works, moving the base pays: joint travel 11.7 -> 7.8 rad, minimum
  sigma_min(J_red) 0.115 -> 0.172.
- **The +/- pi convention acts as a path constraint, not as a filter on the dots.** S+ W+ E- and S- W+ E+ are
  mostly green yet have no schedule, because a joint has to cross 180 deg along the path.
- **Three kinds of obstacle in the (s, x) plane.** Reach limits are the band edges (elbow family). The shoulder
  hole is an interior slit that appears only when the tool leans, and it can cut a band in two (S+ W+ E+ on the
  tilted path). The +/- pi seam is invisible in the dots and acts on the edges between them.
- **Branch-aware planning vs greedy.** With a fast base both complete the same branches, but greedy is 10–46 %
  more expensive and either rides the reach boundary or wanders. With a base slower than about 0.6x the tool
  speed, greedy fails where the DP succeeds: the base must commit early to pass above a grey wedge, and no
  one-step rule can see it.
- **The map depends on the path, not only on the robot.** On the tilted path the feasible branches drop from six
  to three, the best branch changes from S+ W- E+ to S+ W- E-, the parked window shrinks from +/- 0.10 m to
  x = 0, and the joints that hit the seam change (q6 now on two branches). A branch cannot be chosen once for
  all tasks.
- **Singular roots confirm 2.3 on a real path.** On the flat path no singularity is met. On the tilted path all
  elbow-singular dots and most shoulder-singular dots keep sigma_min(J) above the threshold, so the slide supplies
  the lost direction; only 8 dots on the slit are singular for the whole body.
- **The planner is fully analytical.** Every configuration on a schedule is a closed-form IK root; the Jacobian
  enters only through the sigma_min check.

## Conclusion

**How does the base position x select among the IK branches?** With x fixed, the P6R is a UR12e with a shifted
target. The analytical IK (section 3) splits into three groups, which are the three blocks of 3J5 (2.2). Each group
has two roots, so there are at most 8 branches, and the places where a pair of roots merges are exactly the zeros
of det J_red = a2 a3 sin q3 sin q5 rho. Moving the base translates these critical curves in the world. A world
target therefore gains or loses branches as x changes, and always in pairs. For the pose of 1.4 there are 0
branches below x = 0.13 m, 4 up to 0.38 m, and 8 above. On the maps (section 4), a third of a 3 m task strip is
covered by one base position only. The redundancy also matters for singularities: the slide restores rank for
most shoulder and elbow singular configurations (88–96 % of the singular cells), but for only about 12 % of the
wrist ones, because a prismatic joint cannot supply a lost rotation (2.3, 4.4).

**How can x and the branch be planned jointly?** For a Cartesian path, every branch defines its own admissible
region in the (s, x) plane (section 5). A continuous execution is a curve through that region whose slope
dx/ds is bounded by the base speed. Parked at the start position x(0) = -1 m, every branch stops by s = 0.24. A
parked base works only in a 20 cm corridor around x = 0, on three branches. Two branches have no schedule at all,
because a joint must cross the ±π seam. Dynamic programming over the (s, x) graph finds a complete schedule on 6 of
8 branches. Greedy continuation, which picks x one step at a time, completes the same branches when the base is
fast, but costs 10–46 % more and either runs along the reach boundary (sigma_min down to 0.033) or moves the base
far more than needed. When the base is slower than about 0.6× the tool speed, greedy fails where the DP succeeds:
the base must start moving early to pass above a non-admissible wedge, and no one-step rule can see the wedge
coming. The (s, x) map also depends on the task: on the same rectangle tilted by 30/20 deg, only three branches keep a
schedule, the best branch changes, and the shoulder hole appears as an interior slit that can split a
branch's admissible region in two. The branch therefore has to be chosen per path, together with x(s).

## Limitations

- **No collision model.** The IK and the maps check reachability and regularity only. For the pose of 1.4, two of
  the 8 solutions pass through the UMR deck (3.7).
- **Joint limits ±π.** The ±π limits are a project choice, agreed with the TA. They give one candidate per branch but
  create the ±π seam that removes two branches from the path. The real UR12e allows ±2π on every joint except
  the elbow (±π), so on the real robot those two branches may be feasible (not checked).
- **Mount geometry.** The mount position (`scripts/umr_params.py`) is a placeholder until measured. The older
  `p6r/` package and `sim/umr_ur12e_sim.urdf` use a 0.41 m mount and a different base yaw, and do not match this
  report's model.
- **Base along X only.** The real UMR is omnidirectional (x, y, yaw). Here one redundant coordinate was studied.
- **Singularity threshold.** sigma_min mixes metres and radians, so the threshold is scale-dependent. The
  sensitivity check in 4.4 shows the singular class stays a thin band around the same critical curves.
- **Discretisation.** The path is sampled every 4 cm and x every 5 cm (finer in the speed study), and motion between
  samples is not checked. The planner's cost is joint and base travel, not time, and the base speed enters only as
  the slope bound dx/ds.

## Use of AI

AI assistants were used for drafting and debugging code, writing figure scripts, and editing and merging the
report text. All results were run and checked by us.

## References

1. ENG-654 Kinematics-Grounded Motion Planning for Robots, EPFL LASA, lecture notes, simulators and playgrounds.
   https://epfl-lasa.github.io/eng-654/
2. K. P. Hawkins, *Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms*, Georgia Institute of
   Technology, technical report, 2013.
3. D. L. Pieper, *The Kinematics of Manipulators under Computer Control*, PhD thesis, Stanford University, 1968.
4. K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press,
   2017 (product of exponentials, screw axes, geometric Jacobian).
5. Universal Robots, UR12e kinematic parameters and ROS description package (`ur12e/config/ur12e/*.yaml`,
   `ur12e/urdf/` in this repository).
6. AgileX Robotics UMR datasheet and product pages (sources listed in `scripts/umr_params.py`).

