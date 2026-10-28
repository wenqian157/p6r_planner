# FK validation, P6R (UMR x-slide + UR12e)

Date: 2026-10-28  
Input: x = 0.5 m, q1..q6 = [10, -60, 90, -120, -90, 30] deg

## T_world_flange, URDF chain

| | | | |
|---|---|---|---|
| -0.000000 | -0.342020 | -0.939693 |  1.376941 |
| -0.000000 | -0.939693 |  0.342020 |  0.331465 |
| -1.000000 |  0.000000 |  0.000000 |  0.728989 |
|  0.000000 |  0.000000 |  0.000000 |  1.000000 |

## T_world_flange, DH table

| | | | |
|---|---|---|---|
| -0.000000 | -0.342020 | -0.939693 |  1.376941 |
|  0.000000 | -0.939693 |  0.342020 |  0.331465 |
| -1.000000 |  0.000000 |  0.000000 |  0.728989 |
|  0.000000 |  0.000000 |  0.000000 |  1.000000 |

## T_world_flange, PoE

| | | | |
|---|---|---|---|
| -0.000000 | -0.342020 | -0.939693 |  1.376941 |
|  0.000000 | -0.939693 |  0.342020 |  0.331465 |
| -1.000000 | -0.000000 |  0.000000 |  0.728989 |
|  0.000000 |  0.000000 |  0.000000 |  1.000000 |

## Pairwise errors at the input q

| pair | position [m] | orientation [deg] |
|---|---|---|
| URDF chain vs DH table | 2.54e-16 | 1.14e-14 |
| URDF chain vs PoE | 2.78e-16 | 1.44e-14 |
| DH table vs PoE | 2.72e-16 | 3.17e-15 |

## Worst errors over 1000 random q  (x in [-1.5, 1.5] m, angles in [-pi, pi])

| pair | max position [m] | max orientation [deg] |
|---|---|---|
| URDF chain vs DH table | 6.66e-16 | 3.03e-14 |
| URDF chain vs PoE | 1.28e-15 | 2.87e-14 |
| DH table vs PoE | 1.22e-15 | 2.51e-14 |

Verdict: the three models agree (tolerance 1e-09 m, 1e-06 deg).
