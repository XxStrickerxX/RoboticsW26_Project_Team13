"""Milestone 03 (Team 13): inverse position kinematics by Newton-Raphson.

inverse_jacobian_matrix(q)     -> 4x3 numeric pseudo-inverse of dF/dq at the joint angles q
inverse_kinematics_func(q0, X) -> joint angles that move the end effector to X = [x, y, z]

F(q) = FK(q) - X_desired          (FK = forward_kinematics_func from Milestone 02)
q_new = q_old - pinv(dF/dq) * F   (Tutorial 04 slides 17-35; pseudo-inverse because dF/dq is 3x4)

Positions in metres, frame 0 (top of the base). Simscape/Gazebo world: z + 0.0438 m.
Angles in radians. Requires SymPy and NumPy.
"""
import numpy as np
import sympy as sp

from forward_kinematics import forward_kinematics_func, q1, q2, q3, q4

# The symbolic equations are built once, when this file is loaded (about 20 s),
# then turned into fast numeric functions.
X_SYM = forward_kinematics_func()                         # 3x1 symbolic [x, y, z]
DFDQ_SYM = sp.Matrix.hstack(X_SYM.diff(q1), X_SYM.diff(q2),
                            X_SYM.diff(q3), X_SYM.diff(q4))   # 3x4 symbolic dF/dq
X_NUM = sp.lambdify((q1, q2, q3, q4), X_SYM, 'numpy')
DFDQ_NUM = sp.lambdify((q1, q2, q3, q4), DFDQ_SYM, 'numpy')

TOLERANCE = 1e-6        # m (0.001 mm)
MAX_ITERATIONS = 100


def fk_numeric(q):
    """End-effector position [x, y, z] (m, frame 0) for numeric joint angles q (rad)."""
    return np.array(X_NUM(q[0], q[1], q[2], q[3]), dtype=float).flatten()


def inverse_jacobian_matrix(q):
    """Pseudo-inverse (4x3) of dF/dq evaluated at the numeric joint angles q (rad)."""
    dfdq = np.array(DFDQ_NUM(q[0], q[1], q[2], q[3]), dtype=float)
    return np.linalg.pinv(dfdq)


def inverse_kinematics_func(q0, X):
    """Newton-Raphson: joint angles (rad) that place the end effector at X = [x, y, z] (m)."""
    q = np.array(q0, dtype=float)
    Xd = np.array(X, dtype=float)
    for k in range(1, MAX_ITERATIONS + 1):
        F = fk_numeric(q) - Xd
        if np.linalg.norm(F) < TOLERANCE:
            break
        q = q - inverse_jacobian_matrix(q) @ F
        q = np.mod(q + np.pi, 2 * np.pi) - np.pi      # keep angles in [-180, 180) deg
    print(f'IK: {k} iterations, final error = {np.linalg.norm(F) * 1000:.2e} mm')
    if np.linalg.norm(F) >= TOLERANCE:
        print('IK warning: target not reached, it may be outside the workspace.')
    return q
