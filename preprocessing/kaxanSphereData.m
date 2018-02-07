% kaxanSphereData.m     e.anderlini@ucl.ac.uk     05/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is used to extract the ROV data in the provided folder and
% create a Matlab object saved in a Matlab format. A sphere is added as an
% additional payload.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input data:
dir_name = './Kaxan/';        % directory name
rov.g = 9.81;                 % gravitational constant (m/s2)
rov.density = 1024;           % sea water density (kg/m3)

%% Extract the equations fits:
line_520 = csvread([dir_name,'520.csv']);
line_540 = csvread([dir_name,'540.csv']);
% % Prepare data for fitting & open curve fitting app:
% line_520_x = line_520(:,1);
% line_520_y = line_520(:,2);
% line_540_x = line_540(:,1);
% line_540_y = line_540(:,2);
% 
% line_520_x_1 = line_520_x(1:24);
% line_520_y_1 = abs(line_520_y(1:24));
% line_520_x_2 = line_520_x(25:end);
% line_520_y_2 = line_520_y(25:end);
% line_540_x_1 = line_540_x(1:21);
% line_540_y_1 = abs(line_540_y(1:21));
% line_540_x_2 = line_540_x(22:end);
% line_540_y_2 = line_540_y(22:end);
% line_540_y_2(1:3) = [0.1;0.1;0.1];

% Data after the fit:
theta_520 = [0.6738,0.7566,-0.3969,-1.241];
theta_540 = [0.7696,0.03518,1.256,-0.1574];

theta_520_1 = [-2.684,0.2097,1.473];
theta_520_2 = [0.9795,0.3427,0.2088];
theta_540_1 = [-0.6473,0.4223,-0.4759];
theta_540_2 = [1.003,0.3495,-0.07633];

%% Sphere data:
r = 0.15;
z = 0.5;
ma = 2/3*pi*rov.density*r^3;
ms = 20;
fd = 0.25*pi*rov.density*r^3;
Ma = [ma,0,0,0,ma*z,0;0,ma,0,-ma*z,0,0;0,0,ma,0,0,0;0,-ma*z,0,ma*z^2,0,0;...
    ma*z,0,0,0,ma*z^2,0;0,0,0,0,0,ma*z^2];
Ms = [ms,0,0,0,ms*z,0;0,ms,0,-ms*z,0,0;0,0,ms,0,0,0;0,-ms*z,0,ms*z^2,0,0;...
    ms*z,0,0,0,ms*z^2,0;0,0,0,0,0,ms*z^2];
Dq = [fd,0,0,0,fd*z,0;0,fd,0,-fd*z,0,0;0,0,fd,0,0,0;0,-fd*z,0,fd*z^2,0,0;...
    fd*z,0,0,0,fd*z^2,0;0,0,0,0,0,fd*z^2];

%% Input data for Kaxan ROV:
rov.m        = 98.5;
rov.I        = [1.32,0,0;0,2.08,0;0,0,2.32];
rov.CoB      = [0;0;-0.1];
rov.CoG      = [0;0;0];
rov.M_B      = [diag(rov.m.*[1;1;1]),zeros(3);zeros(3),rov.I];
rov.M_A      = diag([29;30;90;5.2;7.2;3.3]);
rov.D_l      = diag([72;77;95;40;30;30]);
rov.D_q      = zeros(6);
rov.F        = [0,-0.175,-0.10;0,0.215,-0.1;0.135,0,0.07;-0.022,0,0];
rov.coeffs   = [theta_520;theta_540];
rov.inv_coeffs = [theta_520_1;theta_520_2;theta_540_1;theta_540_2];
rov.O        = zeros(3);

%% Merging of ROV and sphere data:
rov.M_B = rov.M_B+Ms;
rov.M_A = rov.M_A+Ma;
rov.D_l = rov.D_l+Dq;

%% Calculate the remaining data:
rov.CoB(3) = (rov.CoB(3)*rov.m+z*2*ma)/(rov.m+2*ma);
rov.CoG(3) = (rov.CoG(3)*rov.m+z*ms)/(ms+rov.m);
rov.volume   = (2*ma+rov.m)/rov.density;
rov.weight   = (rov.m+ms)*rov.g;
rov.buoyancy = (2*ma+rov.m)*rov.g;
rov.M        = rov.M_B+rov.M_A;
rov.inv_mass = pinv(rov.M);
rov.S_r_g    = skew(rov.CoG);
rov.T        = [1,1,0,0;0,0,1,0;0,0,0,1;0,0,-rov.F(3,3),rov.F(4,2);...
    rov.F(1,3),rov.F(2,3),0,-rov.F(4,1);-rov.F(1,2),-rov.F(2,2),...
    rov.F(3,1),0];

%% Store the generated data in a Matlab file:
save('../data/rov_sphere.mat','rov');