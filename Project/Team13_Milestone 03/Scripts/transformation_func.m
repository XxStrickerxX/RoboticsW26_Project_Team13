function T = transformation_func(theta, d, a, alpha)
% TRANSFORMATION_FUNC  DH homogeneous transformation ^{i-1}T_i for one row
% of the DH table (Tutorial 03 slide 13, Lecture 03 slides 21-22):
%   T = Rot(z,theta) * Trans(z,d) * Trans(x,a) * Rot(x,alpha)
% theta can be symbolic (joint variable, e.g. q1 or q2 - pi/2);
% d, a, alpha are constants. Angles are in radians.

    % Convert to sym so cos(pi/2) etc. evaluate to exact 0/1 instead of 6e-17
    theta = sym(theta);
    alpha = sym(alpha);
    d     = sym(d);
    a     = sym(a);

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
