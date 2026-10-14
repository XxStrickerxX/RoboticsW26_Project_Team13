function [X, T0E] = forward_kinematics_func()
% FORWARD_KINEMATICS_FUNC  Forward position kinematics of our 4-DoF desktop arm.
% Calls transformation_func once per DH row, multiplies the matrices
% (Tutorial 03 slide 14: ^0T_E = ^0T_1 * ^1T_2 * ... ) and extracts the
% end-effector position.
%   X   : 3x1 symbolic [x; y; z] of the end effector w.r.t. the base frame
%   T0E : 4x4 symbolic total homogeneous transformation (base -> end effector)

    syms q1 q2 q3 q4 real

    % ------------------------------ DH TABLE ------------------------------
    % Source: course URDF (robot.urdf.xacro), see DECISIONS.md D7/D7a/D7b.
    % Frame 0 on top of the base (Base_Link), so Simscape/Gazebo world Z = our Z + 0.0438 m.
    % Lengths in metres, angles in radians. Each row: [theta_i, d_i, a_i, alpha_i]
    % Constants written as exact decimals (sym('...')) so the printed equations stay readable
    c = @(s) sym(s);
    DH = [ q1,                  c('0.04975'), 0,             pi/2 ;   % joint 1 : frame 0 -> 1 (waist)
           q2 + pi/2,           0,            c('0.140'),    pi   ;   % joint 2 : frame 1 -> 2 (shoulder)
           q3 - c('0.012434'),  0,            c('0.132900'), pi   ;   % joint 3 : frame 2 -> 3 (elbow)
           q4 - c('0.082320'),  c('0.01226'), c('0.100245'), 0    ];  % joint 4 : frame 3 -> 4 (wrist -> EE_frame)
    % -----------------------------------------------------------------------

    T0E = sym(eye(4));
    for i = 1:size(DH, 1)
        T0E = T0E * transformation_func(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    end
    T0E = simplify(T0E);

    X = T0E(1:3, 4);
end
