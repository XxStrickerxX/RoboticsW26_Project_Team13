function J = jacobian_matrix(q)
% JACOBIAN_MATRIX  Symbolic 6x4 Jacobian of our 4-DoF arm (Milestone 03).
%   q : 4x1 vector of joint angles (symbolic, e.g. [q1; q2; q3; q4])
%   J : 6x4 symbolic Jacobian,  [v; w] = J * q_dot   (Tutorial 05 slides 8-10)
% Rows 1-3 (Jv): differentiate the end-effector position x, y, z with respect to each
%                joint angle (differentiation approach, Tutorial 05 slides 4-8).
% Rows 4-6 (Jw): all joints are revolute, so column i is the z axis of frame i-1
%                (general approach, Tutorial 05 slides 10-11).

    syms q1 q2 q3 q4 real
    [X, T04, T01, T02, T03] = forward_kinematics_func();

    % ----- linear velocity part: dX/dq -----
    Jv = [diff(X, q1), diff(X, q2), diff(X, q3), diff(X, q4)];

    % ----- angular velocity part: z axes of frames 0, 1, 2, 3 -----
    z0 = [0; 0; 1];
    z1 = T01(1:3, 3);
    z2 = T02(1:3, 3);
    z3 = T03(1:3, 3);
    Jw = [z0, z1, z2, z3];

    J = simplify([Jv; Jw]);

    % put in the joint angles that were passed in (symbols or numbers)
    J = subs(J, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]);
end
