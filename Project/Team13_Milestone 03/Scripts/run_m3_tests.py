"""Milestone 03 tests in Python (Team 13): same cases as run_m3_tests.m.
All four joints move in every case. Positions in frame 0 (top of the base).
Run:  python3 run_m3_tests.py
"""
import numpy as np

from inverse_kinematics import fk_numeric, inverse_kinematics_func
from velocity_kinematics import forward_velocity_numeric, inverse_velocity_kinematics

# desired EE positions (frame 0, mm); each one needs all four joints to move
X_CASES_MM = [[150.0, 120.0, 250.0],
              [-180.0, 100.0, 200.0],
              [-100.0, -200.0, 150.0]]
V_CASES = [[0.020, -0.010, 0.015],
           [-0.015, 0.020, 0.010],
           [0.010, 0.015, -0.020]]
Q0 = [0.0, -0.5, 1.0, -0.5]        # bent pose, away from the straight-up singularity

for c in range(3):
    print(f'\n================ Case {c + 1} ================')
    Xd = np.array(X_CASES_MM[c]) / 1000.0
    print('Desired EE position (mm):     ', np.round(Xd * 1000, 3))
    q = inverse_kinematics_func(Q0, Xd)
    print('IK joint angles (deg):        ', np.round(np.degrees(q), 3))
    X_check = fk_numeric(q)
    print('FK of the IK angles (mm):     ', np.round(X_check * 1000, 3))
    print(f'Position error (mm):           {np.linalg.norm(X_check - Xd) * 1000:.2e}')

    Vd = np.array(V_CASES[c])
    q_dot = inverse_velocity_kinematics(q, Vd)
    print('Desired EE velocity (mm/s):   ', np.round(Vd * 1000, 3))
    print('IVK joint velocities (deg/s): ', np.round(np.degrees(q_dot), 3))
    V_F = forward_velocity_numeric(q, q_dot)
    print('FVK linear velocity (mm/s):   ', np.round(V_F[0:3] * 1000, 3))
    print('FVK angular velocity (deg/s): ', np.round(np.degrees(V_F[3:6]), 3))
    print(f'Velocity error (mm/s):         {np.linalg.norm(V_F[0:3] - Vd) * 1000:.2e}')
