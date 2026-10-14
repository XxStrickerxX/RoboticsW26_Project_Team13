% RUN_FK_TESTS  Simulates robot_fk_test.slx for the M2 test cases (DECISIONS D8) and
% compares the Simscape end-effector position with our DH forward kinematics.
% Both are in the world frame (ground); FK frame-0 z is shifted by +0.0438 m inside the model.

mdl = 'robot_fk_test';
load_system(mdl);
cases = { 'Case A: all zeros',         [0 0 0 0];
          'Case B: [30 20 -40 25] deg', [30 20 -40 25] };
for k = 1:size(cases, 1)
    q_deg = cases{k, 2};
    assignin('base', 'q_deg', q_deg);
    out = sim(mdl, 'SrcWorkspace', 'base');
    s = out.ee_sim(end, :);                       % Mux output: N x 3
    f = squeeze(out.ee_fk); f = f(:, end).';      % MATLAB Function output: 3 x 1 x N
    fprintf('\n%s\n', cases{k, 1});
    fprintf('  Simscape EE (world, m): x=%9.5f  y=%9.5f  z=%9.5f\n', s);
    fprintf('  FK (DH)  EE (world, m): x=%9.5f  y=%9.5f  z=%9.5f\n', f);
    fprintf('  error (mm): %.4f  %.4f  %.4f   |e| = %.4f mm\n', (s - f) * 1e3, norm(s - f) * 1e3);
end
