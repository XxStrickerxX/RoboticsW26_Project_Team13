% RUN_M3_SIMULINK_TESTS  Milestone 03 Simulink/Simscape tests (Team 13).
% For each case: desired EE position X_d and velocity V_d -> IK and IVK blocks ->
% joints q_i(t) = q_i + qd_i*t in Simscape -> sensed EE position and velocity,
% compared with the FK / FVK equations and with the desired values.
% All four joints move in every case.

clear; clc;
mdl = 'robot_m3_test';
load_system(mdl);

% desired positions (frame 0), one column per case, in mm
X_cases = [150  -180  -100;
           120   100  -200;
           250   200   150] / 1000;          % mm -> m
V_cases = [ 0.020, -0.015,  0.010;
           -0.010,  0.020,  0.015;
            0.015,  0.010, -0.020];

for c = 1:3
    X_d = X_cases(:, c);
    V_d = V_cases(:, c);
    out = sim(mdl);

    sim_pos = out.sim_pos;   fk_pos = out.fk_pos;
    sim_vel = out.sim_vel;   fvk_vel = out.fvk_vel;
    q_ik = out.q_ik(end, :); qd_ivk = out.qd_ivk(end, :);

    fprintf('\n================ Case %d ================\n', c);
    fprintf('Desired EE position (frame 0, mm):  %9.3f %9.3f %9.3f\n', X_d * 1000);
    fprintf('Desired EE velocity (mm/s):         %9.3f %9.3f %9.3f\n', V_d * 1000);
    fprintf('IK joint angles (deg):              %9.3f %9.3f %9.3f %9.3f\n', q_ik * 180 / pi);
    fprintf('IVK joint velocities (deg/s):       %9.3f %9.3f %9.3f %9.3f\n', qd_ivk * 180 / pi);
    % the same angles from our MATLAB functions (called once in the model's InitFcn)
    fprintf('MATLAB functions vs blocks: max |q| difference = %.1e rad, max |qd| difference = %.1e rad/s\n', ...
            max(abs(out.fun_diff(end, 1:4))), max(abs(out.fun_diff(end, 5:8))));

    % at t = 0 the arm is at the IK solution and moves with the IVK velocities
    fprintf('t = 0: Simscape EE (world, mm):     %9.3f %9.3f %9.3f\n', sim_pos(1, :) * 1000);
    fprintf('t = 0: desired EE (world, mm):      %9.3f %9.3f %9.3f\n', (X_d + [0; 0; 0.0438]) * 1000);
    fprintf('t = 0: position error (mm):         %.4f\n', norm(sim_pos(1, :)' - X_d - [0; 0; 0.0438]) * 1000);
    fprintf('t = 0: Simscape EE velocity (mm/s): %9.3f %9.3f %9.3f\n', sim_vel(1, 1:3) * 1000);
    fprintf('t = 0: velocity error (mm/s):       %.4f\n', norm(sim_vel(1, 1:3)' - V_d) * 1000);

    % during the whole motion: Simscape vs the FK / FVK equations
    e_pos = 0;  e_vel = 0;  e_w = 0;
    for k = 1:size(sim_pos, 1)
        e_pos = max(e_pos, norm(sim_pos(k, :) - fk_pos(k, :)));
        e_vel = max(e_vel, norm(sim_vel(k, 1:3) - fvk_vel(k, 1:3)));
        e_w   = max(e_w,   norm(sim_vel(k, 4:6) - fvk_vel(k, 4:6)));
    end
    fprintf('0-2 s: max |Simscape - FK| position (mm):          %.4f\n', e_pos * 1000);
    fprintf('0-2 s: max |Simscape - FVK| linear velocity (mm/s): %.4f\n', e_vel * 1000);
    fprintf('0-2 s: max |Simscape - FVK| angular vel. (deg/s):  %.5f\n', e_w * 180 / pi);
end
