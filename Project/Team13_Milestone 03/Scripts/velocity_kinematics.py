"""Milestone 03 (Team 13): forward and inverse velocity kinematics.

jacobian_matrix(q)                      -> symbolic 6x4 Jacobian (Tutorial 05)
forward_velocity_kinematics(q, q_dot)   -> V_F = J(q) * q_dot   [vx, vy, vz, wx, wy, wz]
inverse_velocity_kinematics(q, V_F)     -> q_dot = J_inv(q) * [vx, vy, vz]  (numeric)

Rows 1-3 of J: derivatives of x, y, z w.r.t. each joint (differentiation approach, T05 sl.4-8).
Rows 4-6 of J: z axes of frames 0..3, since every joint is revolute (general approach, T05 sl.10-11).
"""
import numpy as np
import sympy as sp

from forward_kinematics import DH_TABLE, transformation_func, q1, q2, q3, q4
from inverse_kinematics import X_SYM, inverse_jacobian_matrix

Q = (q1, q2, q3, q4)


def jacobian_matrix(q):
    """Symbolic 6x4 Jacobian with the joint angles q put in (symbols or numbers)."""
    # linear part: differentiate the FK position
    Jv = sp.Matrix.hstack(X_SYM.diff(q1), X_SYM.diff(q2), X_SYM.diff(q3), X_SYM.diff(q4))
    # angular part: z axis (3rd column of the rotation) of frames 0, 1, 2, 3
    T = sp.eye(4)
    z_axes = [sp.Matrix([0, 0, 1])]
    for theta, d, a, alpha in DH_TABLE[:3]:
        T = T * transformation_func(theta, d, a, alpha)
        z_axes.append(sp.simplify(T[0:3, 2]))
    Jw = sp.Matrix.hstack(*z_axes)
    J = sp.Matrix.vstack(Jv, Jw)
    return J.subs(dict(zip(Q, q)))


def forward_velocity_kinematics(q, q_dot):
    """V_F = J(q) * q_dot (symbolic if q / q_dot are symbols)."""
    return jacobian_matrix(q) * sp.Matrix(q_dot)


# numeric version of J for fast use inside the ROS nodes (built once)
q_dot_syms = sp.symbols('qd1 qd2 qd3 qd4', real=True)
_FVK_NUM = sp.lambdify((Q, q_dot_syms), forward_velocity_kinematics(Q, q_dot_syms), 'numpy')


def forward_velocity_numeric(q, q_dot):
    """Numeric V_F = [vx, vy, vz, wx, wy, wz] for numeric q (rad) and q_dot (rad/s)."""
    return np.array(_FVK_NUM(tuple(q), tuple(q_dot)), dtype=float).flatten()


def inverse_velocity_kinematics(q, V_F):
    """Joint velocities (rad/s) for the desired linear EE velocity V_F = [vx, vy, vz] (m/s)."""
    return inverse_jacobian_matrix(q) @ np.array(V_F[0:3], dtype=float)
