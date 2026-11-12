% RUN_M3_TESTS  Milestone 03 tests in MATLAB (Team 13).
% 1) Inverse position kinematics (Newton-Raphson) validated with the forward kinematics.
% 2) Inverse velocity kinematics validated with the forward velocity kinematics.
% Every test case moves all four joints.
% Positions are in frame 0 (top of the base). Simscape/Gazebo world: z + 0.0438 m.

clear; clc;
syms q1 q2 q3 q4 real
X_fk = forward_kinematics_func();

% ---------------- test cases ----------------
% Desired end-effector positions (frame 0), one column per case, given in mm.
% Each one needs all four joints to move (checked in the output below).
X_targets = [150  -180  -100;
             120   100  -200;
             250   200   150] / 1000;          % mm -> m

% Desired end-effector linear velocities (m/s, frame 0), one per case
V_targets = [ 0.020, -0.015,  0.010;
             -0.010,  0.020,  0.015;
              0.015,  0.010, -0.020];

q0 = [0; -0.5; 1.0; -0.5];         % initial guess (rad), a bent pose away from the
                                   % straight-up singular pose; same for all cases

for c = 1:3
    fprintf('\n================ Case %d ================\n', c);

    Xd = X_targets(:, c);
    fprintf('Desired EE position (mm):        x=%9.3f  y=%9.3f  z=%9.3f\n', Xd * 1000);

    % ---- 1) inverse position kinematics ----
    q = inverse_kinematics_func(q0, Xd);
    fprintf('IK joint angles (deg):           q1=%8.3f q2=%8.3f q3=%8.3f q4=%8.3f\n', q * 180 / pi);

    % validate: put the IK angles back into the forward kinematics
    X_check = double(subs(X_fk, [q1, q2, q3, q4], [q(1), q(2), q(3), q(4)]));
    fprintf('FK of the IK angles (mm):        x=%9.3f  y=%9.3f  z=%9.3f\n', X_check * 1000);
    fprintf('Position error (mm):             %.2e\n', norm(X_check - Xd) * 1000);

    % ---- 2) inverse velocity kinematics ----
    Vd = V_targets(:, c);
    q_dot = inverse_velocity_kinematics(q, Vd);
    fprintf('Desired EE velocity (mm/s):      vx=%8.3f vy=%8.3f vz=%8.3f\n', Vd * 1000);
    fprintf('IVK joint velocities (deg/s):    %8.3f %8.3f %8.3f %8.3f\n', q_dot * 180 / pi);

    % validate: put q and q_dot into the forward velocity kinematics
    V_F = double(forward_velocity_kinematics(q, q_dot));
    fprintf('FVK linear velocity (mm/s):      vx=%8.3f vy=%8.3f vz=%8.3f\n', V_F(1:3) * 1000);
    fprintf('FVK angular velocity (deg/s):    wx=%8.3f wy=%8.3f wz=%8.3f\n', V_F(4:6) * 180 / pi);
    fprintf('Velocity error (mm/s):           %.2e\n', norm(V_F(1:3) - Vd) * 1000);

    % independent check of the Jacobian: move the joints a tiny step and
    % divide the change in position by the time step (numerical derivative)
    dt = 1e-6;
    q_plus  = q + q_dot * dt;
    q_minus = q - q_dot * dt;
    X_plus  = double(subs(X_fk, [q1, q2, q3, q4], [q_plus(1), q_plus(2), q_plus(3), q_plus(4)]));
    X_minus = double(subs(X_fk, [q1, q2, q3, q4], [q_minus(1), q_minus(2), q_minus(3), q_minus(4)]));
    V_numeric = (X_plus - X_minus) / (2 * dt);
    fprintf('Numerical derivative check (mm/s): %.2e\n', norm(V_numeric - V_F(1:3)) * 1000);
end
