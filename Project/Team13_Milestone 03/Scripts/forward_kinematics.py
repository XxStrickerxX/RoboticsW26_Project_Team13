"""Forward position kinematics of the Team 13 4-DoF desktop arm (Milestone 02).

transformation_func(theta, d, a, alpha): DH link matrix ^{i-1}T_i (Tutorial 03 slide 13).
forward_kinematics_func(): multiplies the 4 DH matrices (Tutorial 03 slide 14) and
returns the symbolic end-effector position X = [x, y, z] w.r.t. frame 0.

Frame 0 is on top of the base (Base_Link). Simscape/Gazebo world Z = our Z + 0.0438 m.
Lengths in metres, angles in radians. Requires SymPy (pip install sympy).
"""
import sympy as sp

q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4', real=True)

# DH table from the course URDF (DECISIONS.md D7a-D7c): [theta, d, a, alpha]
DH_TABLE = [
    [q1,                                  sp.Rational('0.04975'), 0,                       sp.pi / 2],
    [q2 + sp.pi / 2,                      0,                      sp.Rational('0.140'),    sp.pi],
    [q3 - sp.Rational('0.012434'),        0,                      sp.Rational('0.132900'), sp.pi],
    [q4 - sp.Rational('0.082320'),        sp.Rational('0.01226'), sp.Rational('0.100245'), 0],
]

BASE_HEIGHT = 0.0438  # Base_Link above the world/ground frame (m)


def transformation_func(theta, d, a, alpha):
    """DH homogeneous transformation: Rot(z,theta) Trans(z,d) Trans(x,a) Rot(x,alpha)."""
    theta, d, a, alpha = (sp.sympify(v) for v in (theta, d, a, alpha))
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0,              sp.sin(alpha),                  sp.cos(alpha),                 d],
        [0,              0,                              0,                             1],
    ])


def total_transformation():
    """^0T_E = ^0T_1 * ^1T_2 * ^2T_3 * ^3T_4 (symbolic)."""
    T = sp.eye(4)
    for theta, d, a, alpha in DH_TABLE:
        T = T * transformation_func(theta, d, a, alpha)
    return sp.simplify(T)


def forward_kinematics_func():
    """Symbolic end-effector position [x, y, z] w.r.t. frame 0 (top of base)."""
    return total_transformation()[0:3, 3]


def evaluate(q_values):
    """Numeric EE position (m) for joint angles q_values = [q1, q2, q3, q4] in rad."""
    X = forward_kinematics_func()
    return [float(v) for v in X.subs(dict(zip((q1, q2, q3, q4), q_values)))]


if __name__ == '__main__':
    X = forward_kinematics_func()
    for name, expr in zip('xyz', X):
        print(f'{name} = {expr}\n')
    for q in ([0, 0, 0, 0], [0.5236, 0.3491, -0.6981, 0.4363]):
        p = evaluate(q)
        print(f'q = {q} -> frame0 [m] = {[round(v, 5) for v in p]}, '
              f'world [m] = {[round(p[0], 5), round(p[1], 5), round(p[2] + BASE_HEIGHT, 5)]}')
