function q_dot = inverse_velocity_kinematics(q, V_F)
% INVERSE_VELOCITY_KINEMATICS  Joint velocities for a desired end-effector velocity (Milestone 03).
%   q     : 4x1 numeric joint angles (rad)
%   V_F   : desired end-effector linear velocity [vx; vy; vz] (m/s, frame 0 axes)
%   q_dot : 4x1 joint velocities (rad/s)
% Inverse velocity kinematics:  q_dot = J_inv(q) * V_F
% J_inv is the pseudo-inverse of the 3x4 position Jacobian (4 joints, 3 velocity
% components), from inverse_jacobian_matrix.

    J_inv = inverse_jacobian_matrix(q);
    q_dot = J_inv * [V_F(1); V_F(2); V_F(3)];
end
