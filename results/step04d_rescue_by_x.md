# Step 4D: can the slide rescue an arm singularity?

Script: `scripts/step04d_rescue_by_x.py`

```text
====================================================================================================
elbow     yaw 0   :  x = 0.5 m,  q = [  0 -45   0 -45 -90   0] deg
  sigma_min(J_red) = 3.72e-18     sigma_min(J) = 0.4472
  lost direction u = [ 0.6918  0.1259  0.6918 -0.0147  0.1635 -0.    ]   [vx vy vz wx wy wz]     |u_x| = 0.6918

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

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
====================================================================================================
elbow     vertical:  x = 0.5 m,  q = [  0 -90   0   0 -90   0] deg
  sigma_min(J_red) = 7.82e-17     sigma_min(J) = 0.0000
  lost direction u = [ 0.      0.      0.9929  0.      0.119  -0.    ]   [vx vy vz wx wy wz]     |u_x| = 0.0000

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

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
====================================================================================================
wrist     yaw 90  :  x = 0.5 m,  q = [  90  -60   90 -120    0   30] deg
  sigma_min(J_red) = 5.42e-17     sigma_min(J) = 0.3426
  lost direction u = [ 0.7355 -0.      0.     -0.      0.      0.6775]   [vx vy vz wx wy wz]     |u_x| = 0.7355

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 0.0000,   base speed xdot = 1.3596 m/s
          -> the slide supplies what the arm cannot

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q5)
           x - x0 [m]  pose error  held?  sigma_min(J_red)     q5 [deg]   q [deg]
               -0.300    3.91e-12    yes            0.2379      18.5755   [ 71.4 -45.1  77.8 147.3  18.6 120. ]
               -0.250    3.67e-12    yes            0.2001      15.5674   [ 74.4 -45.5  78.7 146.9  15.6 120. ]
               -0.200    3.57e-12    yes            0.1611      12.5005   [ 77.5 -45.8  79.1 146.7  12.5 120. ]
               -0.150    3.60e-12    yes            0.1211       9.3919   [ 80.6 -45.8  79.1 146.7   9.4 120. ]
               -0.100    3.75e-12    yes            0.0807       6.2598   [ 83.7 -45.6  78.7 146.9   6.3 120. ]
               -0.050    5.89e-10    yes            0.0402       3.1229   [ 86.9 -45.1  77.9 147.3   3.1 120. ]
                0.000    7.31e-10    yes            0.0000       0.0000   [  90.  -60.   90. -120.    0.   30.]
                0.050    4.20e-10    yes            0.0340       3.0911   [ 93.1 -52.5  64.7 -12.2   3.1 -60. ]
                0.100    6.24e-10    yes            0.0661       6.1339   [ 96.1 -51.1  62.4 -11.3   6.1 -60. ]
                0.150    6.29e-10    yes            0.0951       9.1136   [ 99.1 -49.5  59.6 -10.2   9.1 -60. ]
                0.200    1.10e-10    yes            0.1193      12.0176   [102.  -47.5  56.2  -8.8  12.  -60. ]
                0.250    1.16e-10    yes            0.1364      14.8354   [104.8 -45.2  52.2  -7.   14.8 -60. ]
                0.300    1.78e-10    yes            0.1439      17.5590   [107.6 -42.5  47.4  -4.9  17.6 -60. ]
          -> rescued for x - x0 in {-0.30 .. +0.30} m: pose held and the arm is regular
====================================================================================================
wrist     yaw 0   :  x = 0.5 m,  q = [   0  -60   90 -120    0   30] deg
  sigma_min(J_red) = 4.36e-17     sigma_min(J) = 0.0000
  lost direction u = [-0.     -0.7355 -0.      0.     -0.      0.6775]   [vx vy vz wx wy wz]     |u_x| = 0.0000

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 1.0000,   base speed xdot = -0.0000 m/s
          -> the slide cannot help either

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q5)
           x - x0 [m]  pose error  held?  sigma_min(J_red)     q5 [deg]   q [deg]
               -0.300    1.60e-10    yes            0.0000       0.0000   [  -0.   -28.8   36.2 -109.3    0.    41.8]
               -0.250    9.17e-10    yes            0.0000       0.0000   [  -0.   -35.8   49.2 -112.2    0.    38.9]
               -0.200    7.20e-10    yes            0.0000       0.0000   [  -0.   -41.6   59.5 -114.4    0.    36.5]
               -0.150    7.01e-10    yes            0.0000       0.0000   [  -0.   -46.7   68.3 -116.1    0.    34.5]
               -0.100    3.70e-10    yes            0.0000       0.0000   [  -0.   -51.4   76.1 -117.5    0.    32.7]
               -0.050    2.24e-10    yes            0.0000       0.0000   [  -0.   -55.8   83.3 -118.7    0.    31.2]
                0.000    7.31e-10    yes            0.0000       0.0000   [  -0.  -60.   90. -120.    0.   30.]
                0.050    1.46e-10    yes            0.0000       0.0000   [  -0.   -64.1   96.3 -121.2    0.    29. ]
                0.100    1.16e-10    yes            0.0000       0.0000   [  -0.   -68.2  102.2 -122.     0.    27.9]
                0.150    1.02e-10    yes            0.0000      -0.0000   [  -0.   -72.3  107.8 -122.6   -0.    27. ]
                0.200    9.87e-11    yes            0.0000      -0.0000   [  -0.   -76.4  113.2 -123.2   -0.    26.3]
                0.250    1.05e-10    yes            0.0000      -0.0000   [  -0.   -80.6  118.4 -123.6   -0.    25.7]
                0.300    1.20e-10    yes            0.0000      -0.0000   [  -0.   -84.9  123.4 -123.8   -0.    25.3]
          -> pose held for every x but the arm stays singular: the singular set is invariant under the x self-motion
====================================================================================================
shoulder  yaw 90  :  x = 0.5 m,  q = [ 90.    55.    90.   111.95 -90.     0.  ] deg
  sigma_min(J_red) = 5.39e-17     sigma_min(J) = 0.3410
  lost direction u = [ 0.9933 -0.     -0.      0.      0.1128  0.0261]   [vx vy vz wx wy wz]     |u_x| = 0.9933

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 0.0000,   base speed xdot = 1.0068 m/s
          -> the slide supplies what the arm cannot

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q4)
           x - x0 [m]  pose error  held?  sigma_min(J_red)  r_O5-d4 [m]   q [deg]
               -0.300    1.13e+00     no            0.2279       0.2368   [ -27.7  144.9  126.1    5.6  -77.9 -118.5]
               -0.250    1.15e+00     no            0.2477       0.2754   [ -9.  146.1 121.3   5.  -75.9 -99.3]
               -0.200    1.32e+00     no            0.2668       0.2818   [   5.   -90.3 -109.4  108.7  -76.5  -84.9]
               -0.150    1.11e+00     no            0.0466       0.0132   [-176.   119.   -65.    61.5   99.9  -23.6]
               -0.100    1.15e+00     no            0.2485       0.2771   [   7.8   33.9 -121.1  175.1   75.9   98.1]
               -0.050    9.48e-01     no            0.1233       0.1025   [-125.3   93.6  -38.8   85.5  134.5   -1.8]
                0.000    5.73e-08    yes            0.0001       0.0000   [ 90.  55.  90. 112. -90.   0.]
                0.050    7.44e-12    yes            0.0944       0.0500   [129.   44.5  91.9 123.4 -98.2  38.3]
                0.100    4.88e-10    yes            0.1396       0.1000   [ 140.6   39.7   91.7  130.2 -100.    49.8]
                0.150    9.32e-11    yes            0.1779       0.1500   [ 147.5   35.9   90.8  136.2 -101.    56.8]
                0.200    2.49e-11    yes            0.2125       0.2000   [ 152.3   32.7   89.4  141.8 -101.5   61.6]
                0.250    9.83e-12    yes            0.2438       0.2500   [ 155.8   30.    87.4  147.2 -101.9   65.2]
                0.300    6.35e-12    yes            0.2705       0.3000   [ 158.5   27.8   84.9  152.4 -102.1   67.9]
          -> rescued for x - x0 in {+0.05 .. +0.30} m: pose held and the arm is regular
====================================================================================================
shoulder  yaw 0   :  x = 0.5 m,  q = [  0.    55.    90.   111.95 -90.     0.  ] deg
  sigma_min(J_red) = 7.99e-18     sigma_min(J) = 0.0000
  lost direction u = [ 0.      0.9933  0.     -0.1128  0.     -0.0261]   [vx vy vz wx wy wz]     |u_x| = 0.0000

  test 0  arm frozen, x = [-1.5 -1.  -0.5  0.   0.5  1.   1.5]
          sigma_min(J_red) = [0. 0. 0. 0. 0. 0. 0.]   -> unchanged: J_red does not depend on x

  test 1  produce the task velocity u (|u| = 1):
          arm alone  (J_red qd = u):  best residual = 1.0000
          whole body (J qd = u)    :  best residual = 1.0000,   base speed xdot = 0.0000 m/s
          -> the slide cannot help either

  test 2  hold the tool pose, slide the base, re-solve the arm (IK, start nudged by 0.01 rad on q4)
           x - x0 [m]  pose error  held?  sigma_min(J_red)  r_O5-d4 [m]   q [deg]
               -0.300    3.13e-12    yes            0.2019       0.1727   [ -0.   35.3  88.4 133.3 -90.   -0. ]
               -0.250    3.89e-12    yes            0.1694       0.1305   [ -0.   38.   89.7 129.3 -90.   -0. ]
               -0.200    7.76e-12    yes            0.1362       0.0910   [ -0.   41.   90.5 125.4 -90.   -0. ]
               -0.150    3.75e-11    yes            0.1025       0.0557   [ -0.   44.2  91.  121.7 -90.   -0. ]
               -0.100    7.69e-10    yes            0.0686       0.0267   [ -0.   47.6  91.1 118.2 -90.   -0. ]
               -0.050    9.63e-11    yes            0.0343       0.0070   [ -0.   51.2  90.8 115.  -90.   -0. ]
                0.000    5.77e-08    yes            0.0001       0.0000   [  0.  55.  90. 112. -90.   0.]
                0.050    1.19e-10    yes            0.0344       0.0070   [  0.   58.9  88.8 109.2 -90.    0. ]
                0.100    2.13e-11    yes            0.0687       0.0267   [  0.   62.9  87.3 106.8 -90.    0. ]
                0.150    7.10e-11    yes            0.1027       0.0557   [  0.   67.   85.3 104.7 -90.    0. ]
                0.200    1.91e-11    yes            0.1364       0.0910   [  0.   71.2  82.9 102.9 -90.    0. ]
                0.250    1.03e-11    yes            0.1693       0.1305   [  0.   75.5  80.1 101.4 -90.    0. ]
                0.300    8.47e-12    yes            0.2006       0.1727   [  0.   79.9  76.8 100.3 -90.    0. ]
          -> rescued for x - x0 in {-0.30 .. +0.30} m: pose held and the arm is regular
====================================================================================================
```
