% Prints the symbolic X, Y, Z of the end effector (to paste into the Simulink
% MATLAB Function block, M2 req. 5 "From MATLAB to Simulink") and evaluates the
% two test cases. Frame 0 = top of base; Simscape/Gazebo world Z = Z + 0.0438 m.

syms q1 q2 q3 q4 real
[X, T0E] = forward_kinematics_func();

% vpa(...,5): show constants as 5-significant-digit decimals instead of exact fractions
Xd = vpa(X, 5);
fprintf('x = %s\n\n', char(Xd(1)));
fprintf('y = %s\n\n', char(Xd(2)));
fprintf('z = %s\n\n', char(Xd(3)));
disp('0T4 (total homogeneous transformation, base -> end effector):'); disp(vpa(T0E, 5));

% Ready-to-paste MATLAB code for the Simulink MATLAB Function block
fprintf('--- Simulink MATLAB Function body ---\n');
fprintf('x = %s;\ny = %s;\nz = %s;\n\n', char(Xd(1)), char(Xd(2)), char(Xd(3)));

cases = { 'Case A: all zeros', [0 0 0 0];
          'Case B: [30 20 -40 25] deg', deg2rad([30 20 -40 25]) };
for k = 1:size(cases, 1)
    p = double(subs(X, [q1 q2 q3 q4], cases{k, 2}));
    fprintf('%s -> frame0 [m]: x=%.5f y=%.5f z=%.5f | world z=%.5f\n', ...
            cases{k, 1}, p(1), p(2), p(3), p(3) + 0.0438);
end
% Expected (independently computed): A -> 0.00535, -0.01226, 0.42264 (world z 0.46644)
%                                    B -> -0.22147, -0.14202, 0.26201 (world z 0.30581)
