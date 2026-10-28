# Geometric Jacobian of the P6R

Date: 2026-10-28  
Script: `scripts/jacobian_p6r.py`  
Convention: lecture 5, twist = [v; w] in the world frame, velocity point = flange origin.  
Column k: revolute [z_k x (p_t - p_k); z_k], prismatic [z_k; 0].

## Full 6 x 7 Jacobian at x = 0.5 m, q = [10, -60, 90, -120, -90, 30] deg

| | x | q1 | q2 | q3 | q4 | q5 | q6 |
|---|---|---|---|---|---|---|---|
| vx |  1.0000 | -0.3315 |  0.1263 | -0.3962 | -0.1148 | -0.0202 |  0.0000 |
| vy |  0.0000 |  0.8769 |  0.0223 | -0.0699 | -0.0202 |  0.1148 |  0.0000 |
| vz |  0.0000 |  0.0000 | -0.9212 | -0.6148 | -0.1198 |  0.0000 | -0.0000 |
| wx |  0.0000 |  0.0000 | -0.1736 | -0.1736 | -0.1736 |  0.9848 | -0.0000 |
| wy |  0.0000 |  0.0000 |  0.9848 |  0.9848 |  0.9848 |  0.1736 |  0.0000 |
| wz |  0.0000 |  1.0000 |  0.0000 |  0.0000 |  0.0000 | -0.0000 | -1.0000 |

Finite-difference check: max |J - J_fd| = 1.74e-10.  
Slide column equals [1 0 0 0 0 0]: True (pure x translation, independent of q).

## Reduced 6 x 6 (x fixed) vs full 6 x 7 at representative configurations

| configuration | x [m], q [deg] | det J_red | sigma_min(J_red) | rank J_red | sigma_min(J) | rank J | lost direction u [v; w] | abs(u_x) |
|---|---|---|---|---|---|---|---|---|
| regular | [0.5, 10, -60, 90, -120, -90, 30] | 3.23e-01 | 3.18e-01 | 6 | 4.84e-01 | 6 | [ 0.8   0.36  0.35 -0.08  0.31 -0.03] | 0.80 |
| wrist sing.  q5 = 0 | [0.5, 10, -60, 90, -120, 0, 30] | 4.08e-17 | 8.94e-17 | 5 | 4.51e-02 | 6 | [ 0.13 -0.72  0.    0.    0.    0.68] | 0.13 |
| elbow sing.  q3 = 0, arm plane = x-z | [0.5, 0, -45, 0, -45, -90, 0] | -4.65e-34 | 3.72e-18 | 5 | 4.47e-01 | 6 | [ 0.69  0.13  0.69 -0.01  0.16 -0.  ] | 0.69 |
| elbow sing.  q3 = 0, arm plane = y-z | [0.5, 90, -45, 0, -45, -90, 0] | -9.30e-34 | 7.50e-17 | 5 | 7.07e-02 | 6 | [-0.13  0.69  0.69 -0.16 -0.01  0.  ] | 0.13 |
| elbow sing.  q3 = 0, arm vertical | [0.5, 0, -90, 0, 0, -90, 0] | -1.02e-18 | 7.82e-17 | 5 | 1.85e-16 | 5 | [ 0.    0.    0.99  0.    0.12 -0.  ] | 0.00 |
| q = 0  (elbow + wrist singular) | [0.5, 0, 0, 0, 0, 0, 0] | 0.00e+00 | 2.08e-17 | 5 | 9.10e-17 | 5 | [ 0.  0. -0.  1. -0.  0.] | 0.00 |

Reading: rank J = 6 with rank J_red = 5 means the slide rescues the lost direction (abs(u_x) > 0). rank J = 5 means the singularity belongs to the whole body: the lost direction has no x component.
