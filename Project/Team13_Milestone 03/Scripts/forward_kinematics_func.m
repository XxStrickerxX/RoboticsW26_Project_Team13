function [X, T04, T01, T02, T03] = forward_kinematics_func()
% FORWARD_KINEMATICS_FUNC  Forward position kinematics of our 4-DoF arm (Milestone 02).
% DH method (Tutorial 03 slides 6-14): one transformation per DH row, then multiply.
%   X   : 3x1 symbolic end-effector position [x; y; z] in frame 0 (top of the base), metres
%   T04 : 4x4 symbolic total transformation from frame 0 to the end effector
%   T01, T02, T03 : transformations from frame 0 to frames 1, 2, 3
%                   (used by the Jacobian in Milestone 03 for the z axes)
% Simscape / Gazebo world frame is on the ground: z_world = z + 0.0438 m.

    syms q1 q2 q3 q4 real

    % DH table (DECISIONS.md D7c):  theta,  d,  a,  alpha   (m, rad)
    % sym('...') keeps each decimal number exact, so the printed equations stay readable
    T01 = transformation_func(q1,                    sym('0.04975'), 0,               pi/2);  % joint 1 (waist)
    T12 = transformation_func(q2 + pi/2,             0,              sym('0.140'),    pi);    % joint 2 (shoulder)
    T23 = transformation_func(q3 - sym('0.012434'),  0,              sym('0.1329'),   pi);    % joint 3 (elbow)
    T34 = transformation_func(q4 - sym('0.082320'),  sym('0.01226'), sym('0.100245'), 0);     % joint 4 (wrist)

    % multiply the link matrices (Tutorial 03 slide 14)
    T02 = simplify(T01 * T12);
    T03 = simplify(T02 * T23);
    T04 = simplify(T03 * T34);

    % end-effector position = last column of T04
    X = T04(1:3, 4);
end
