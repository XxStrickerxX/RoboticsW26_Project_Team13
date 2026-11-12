function V_F = forward_velocity_kinematics(q, q_dot)
% FORWARD_VELOCITY_KINEMATICS  End-effector velocity from joint velocities (Milestone 03).
%   q     : 4x1 joint angles (symbolic or numeric, rad)
%   q_dot : 4x1 joint velocities (symbolic or numeric, rad/s)
%   V_F   : 6x1 [vx; vy; vz; wx; wy; wz]  (m/s and rad/s, frame 0 axes)
% Forward velocity kinematics:  V_F = J(q) * q_dot   (Tutorial 05 slide 8)

    J = jacobian_matrix(q);
    V_F = J * [q_dot(1); q_dot(2); q_dot(3); q_dot(4)];
end
