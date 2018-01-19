clear;
close all;

syms phi theta psi;

% Translational transformation matrix:
R = [cos(psi)*cos(theta),cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi),...
    sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta);...
    cos(theta)*sin(psi),cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta),...
    cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);
    -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];

% Rotational transformation matrix:
T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
    0, cos(phi), -sin(phi);...
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% 6 degrees of freedom generalised transformation matrix:
J = [R,zeros(3,3);zeros(3,3),T];

% Calculate the inverse transpose matrix:
Ji = inv(J);
Jit = Ji';