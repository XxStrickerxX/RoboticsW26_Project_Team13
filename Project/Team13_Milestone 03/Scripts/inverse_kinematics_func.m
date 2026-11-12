function q = inverse_kinematics_func(q0, X)
% INVERSE_KINEMATICS_FUNC  Inverse position kinematics by Newton-Raphson (Milestone 03).
%   q0 : 4x1 initial guess of the joint angles (rad)
%   X  : 3x1 desired end-effector position [x; y; z] in frame 0 (top of the base), metres
%   q  : 4x1 joint angles that move the end effector to X (rad)
% Newton-Raphson (Tutorial 04 slides 17-35):
%   F(q) = FK(q) - X_desired,     q_new = q_old - (dF/dq)^-1 * F(q_old)
% Here (dF/dq)^-1 is the pseudo-inverse from inverse_jacobian_matrix.

    syms q1 q2 q3 q4 real
    X_fk = forward_kinematics_func();          % symbolic FK from Milestone 02

    q  = [q0(1); q0(2); q0(3); q0(4)];
    Xd = [X(1); X(2); X(3)];
    tolerance = 1e-6;                          % stop when the error is below 0.001 mm
    max_iterations = 100;

    for k = 1:max_iterations
        % position reached with the current guess
        X_now = double(subs(X_fk, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]));
        F = X_now - Xd;
        if norm(F) < tolerance
            break
        end
        % Newton-Raphson step
        J_inv = inverse_jacobian_matrix(q);
        q = q - J_inv * F;
        % keep every angle between -180 and 180 deg (joint limits in the URDF);
        % adding or removing full turns gives the same robot pose
        q = mod(q + pi, 2*pi) - pi;
    end

    fprintf('IK: %d iterations, final error = %.2e mm\n', k, norm(F) * 1000);
    if norm(F) >= tolerance
        fprintf('IK warning: target not reached, it may be outside the workspace.\n');
    end
end
