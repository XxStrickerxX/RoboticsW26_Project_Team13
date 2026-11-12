function J_inv = inverse_jacobian_matrix(q)
% INVERSE_JACOBIAN_MATRIX  Numeric pseudo-inverse of dF/dq at the joint angles q (Milestone 03).
%   q     : 4x1 numeric joint angles (rad)
%   J_inv : 4x3 numeric pseudo-inverse of dF/dq
% F = [x; y; z] is the end-effector position from forward_kinematics_func (Milestone 02).
% dF/dq is 3x4 (3 position equations, 4 joints), so it is not square and has no normal
% inverse. We use the pseudo-inverse, as the Milestone 03 brief asks.

    syms q1 q2 q3 q4 real
    X = forward_kinematics_func();

    % differentiate x, y, z with respect to each joint angle -> 3x4 matrix
    dFdq = [diff(X, q1), diff(X, q2), diff(X, q3), diff(X, q4)];

    % put in the numeric joint angles
    dFdq_num = double(subs(dFdq, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]));

    % pseudo-inverse (4x3)
    J_inv = pinv(dFdq_num);
end
