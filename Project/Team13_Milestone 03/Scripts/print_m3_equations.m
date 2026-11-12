% PRINT_M3_EQUATIONS  Prints the symbolic Milestone 03 matrices (Team 13):
%   dF/dq (3x4, used by Newton-Raphson and the inverse velocity kinematics)
%   J     (6x4 Jacobian: linear part dX/dq + angular part z0..z3)
%   V_F = J * q_dot (forward velocity kinematics)
% Output also saved to print_m3_equations_output.txt

clear; clc;
diary('print_m3_equations_output.txt'); diary on

syms q1 q2 q3 q4 qd1 qd2 qd3 qd4 real
q = [q1; q2; q3; q4];
q_dot = [qd1; qd2; qd3; qd4];

J = jacobian_matrix(q);

fprintf('===== Jacobian J (6x4), rows: vx vy vz wx wy wz, columns: joints 1-4 =====\n');
for i = 1:6
    for k = 1:4
        fprintf('J(%d,%d) = %s\n', i, k, char(J(i, k)));
    end
end

fprintf('\n===== dF/dq = rows 1-3 of J (3x4), used by Newton-Raphson =====\n');
fprintf('(its pseudo-inverse J_inv is evaluated numerically at each step)\n');

V_F = forward_velocity_kinematics(q, q_dot);
fprintf('\n===== Forward velocity kinematics V_F = J * q_dot =====\n');
names = {'vx', 'vy', 'vz', 'wx', 'wy', 'wz'};
for i = 1:6
    fprintf('%s = %s\n', names{i}, char(simplify(V_F(i))));
end

diary off
