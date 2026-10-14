% Sanity check of transformation_func against Tutorial 03, Example 1 (RR planar arm).
% Expected (T03 slide 19):
%   x = l2*cos(q1+q2) + l1*cos(q1),  y = l2*sin(q1+q2) + l1*sin(q1),  z = 0

syms q1 q2 real
l1 = 300; l2 = 500;   % any numeric lengths

T01 = transformation_func(q1, 0, l1, 0);
T12 = transformation_func(q2, 0, l2, 0);
T02 = simplify(T01 * T12);

expected = [l2*cos(q1+q2) + l1*cos(q1);
            l2*sin(q1+q2) + l1*sin(q1);
            0];

assert(isequal(simplify(T02(1:3,4) - expected), sym([0;0;0])), 'Position mismatch');

% alpha = pi/2 must give exact zeros (no 6e-17 residue)
T = transformation_func(q1, 10, 0, pi/2);
assert(isequal(T(3,3), sym(0)), 'cos(pi/2) not exact');

disp('transformation_func OK: matches Tutorial 03 Example 1');
disp(T02)
