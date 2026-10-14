% BUILD_FK_TEST_MODEL  Builds robot_fk_test.slx for Milestone 02 (Team 13).
%  - Starts from robot_matlab.slx (smimport of robot_matlab.urdf, the official course URDF).
%  - Drives Joint_1..Joint_4 with constant angles q_deg (degrees, base workspace).
%  - Transform Sensor: World -> EE_frame position (world frame = ground).
%  - MATLAB Function block with our DH forward kinematics equations (frame 0 = top of base),
%    plus 0.0438 m on z to compare in the world frame.
%  - Displays: Simscape EE, FK EE, error (m).
% Run from this folder:  build_fk_test_model

src = 'robot_matlab';
mdl = 'robot_fk_test';
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Scripts'));   % forward_kinematics_func

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
load_system(src);
save_system(src, mdl);                        % copy under the new name
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');

set_param(mdl, 'PreLoadFcn', 'q_deg = [30 20 -40 25];', ...   % default: test case B
               'StopTime', '2', 'SimMechanicsOpenEditorOnUpdate', 'on');   % Mechanics Explorer opens on Run
q_deg = [30 20 -40 25]; %#ok<NASGU>  (also needed now for compiling)
assignin('base', 'q_deg', q_deg);

% ---------- joint angle source: one constant vector, split per joint ----------
add_block('simulink/Sources/Constant', [mdl '/q (rad)'], ...
          'Value', 'deg2rad(q_deg(:))', 'Position', [-420 380 -340 410]);
add_block('simulink/Signal Routing/Demux', [mdl '/Demux q'], ...
          'Outputs', '4', 'Position', [-300 330 -295 460]);
add_line(mdl, 'q (rad)/1', 'Demux q/1');

for i = 1:4
    j  = sprintf('%s/Joint_%d', mdl, i);
    set_param(j, 'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque');
    s2p = sprintf('%s/S2P q%d', mdl, i);
    add_block('nesl_utility/Simulink-PS Converter', s2p, 'Unit', 'rad', ...
              'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '2', ...
              'InputFilterTimeConstant', '0.01', ...
              'Position', [-240 300+40*i -210 320+40*i]);
    add_line(mdl, sprintf('Demux q/%d', i), sprintf('S2P q%d/1', i));
    % connect the converter to the joint's new (still unconnected) motion input port
    jp = get_param(j, 'PortHandles');
    cand = [jp.LConn jp.RConn];
    free = cand(arrayfun(@(p) get_param(p, 'Line') == -1, cand));
    sp = get_param(s2p, 'PortHandles');
    add_line(mdl, sp.RConn(1), free(1), 'autorouting', 'on');
end

% ---------- Simscape measurement: World -> EE_frame ----------
ts = [mdl '/EE Transform Sensor'];
add_block('sm_lib/Frames and Transforms/Transform Sensor', ts, ...
          'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on', 'Position', [700 380 760 460]);
tp = get_param(ts, 'PortHandles');
wp = get_param([mdl '/World'], 'PortHandles');
ep = get_param([mdl '/EE_frame'], 'PortHandles');
add_line(mdl, wp.RConn(1), tp.LConn(1), 'autorouting', 'on');   % B = World
add_line(mdl, ep.LConn(1), tp.RConn(1), 'autorouting', 'on');   % F = EE_frame
add_block('simulink/Signal Routing/Mux', [mdl '/Mux sim'], 'Inputs', '3', 'Position', [900 380 905 460]);
axes_ = 'xyz';
for k = 1:3
    p2s = sprintf('%s/P2S %c', mdl, axes_(k));
    add_block('nesl_utility/PS-Simulink Converter', p2s, 'Unit', 'm', ...
              'Position', [800 360+30*k 830 380+30*k]);
    pp = get_param(p2s, 'PortHandles');
    add_line(mdl, tp.RConn(1 + k), pp.LConn(1), 'autorouting', 'on');
    add_line(mdl, sprintf('P2S %c/1', axes_(k)), sprintf('Mux sim/%d', k));
end

% ---------- our forward kinematics (MATLAB Function block) ----------
syms q1 q2 q3 q4 real
X = forward_kinematics_func();
Xs = arrayfun(@(e) char(vpa(e, 8)), X, 'UniformOutput', false);
code = sprintf(['function p = fk(q)\n' ...
    '%% Forward position kinematics (DH, Team 13). Frame 0 = top of base. Output in m.\n' ...
    'q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);\n' ...
    'x = %s;\ny = %s;\nz = %s;\np = [x; y; z];\n'], Xs{1}, Xs{2}, Xs{3});
fkb = [mdl '/FK equations (DH)'];
add_block('simulink/User-Defined Functions/MATLAB Function', fkb, 'Position', [-200 600 -60 660]);
rt = sfroot; chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', fkb);
chart.Script = code;
add_line(mdl, 'q (rad)/1', 'FK equations (DH)/1', 'autorouting', 'on');
add_block('simulink/Math Operations/Bias', [mdl '/frame0 -> world'], ...
          'Bias', '[0; 0; 0.0438]', 'Position', [0 615 60 645]);
add_line(mdl, 'FK equations (DH)/1', 'frame0 -> world/1');

% ---------- comparison and displays ----------
add_block('simulink/Math Operations/Sum', [mdl '/error'], 'Inputs', '+-', 'Position', [1000 600 1030 630]);
add_line(mdl, 'Mux sim/1', 'error/1', 'autorouting', 'on');
add_line(mdl, 'frame0 -> world/1', 'error/2', 'autorouting', 'on');
add_block('simulink/Sinks/Display', [mdl '/Simscape EE (m)'], 'Position', [1000 400 1100 460], 'Format', 'long');
add_block('simulink/Sinks/Display', [mdl '/FK EE world (m)'], 'Position', [150 600 250 660], 'Format', 'long');
add_block('simulink/Sinks/Display', [mdl '/Error sim-FK (m)'], 'Position', [1080 590 1180 650], 'Format', 'long');
add_line(mdl, 'Mux sim/1', 'Simscape EE (m)/1', 'autorouting', 'on');
add_line(mdl, 'frame0 -> world/1', 'FK EE world (m)/1', 'autorouting', 'on');
add_line(mdl, 'error/1', 'Error sim-FK (m)/1');
add_block('simulink/Sinks/To Workspace', [mdl '/ee_sim'],   'VariableName', 'ee_sim',   'SaveFormat', 'Array', 'Position', [1000 480 1060 500]);
add_block('simulink/Sinks/To Workspace', [mdl '/ee_fk'],    'VariableName', 'ee_fk',    'SaveFormat', 'Array', 'Position', [150 680 210 700]);
add_line(mdl, 'Mux sim/1', 'ee_sim/1', 'autorouting', 'on');
add_line(mdl, 'frame0 -> world/1', 'ee_fk/1', 'autorouting', 'on');

save_system(mdl);
fprintf('Built %s.slx\n', mdl);
