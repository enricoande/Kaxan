% ls_regression.m     e.anderlini@ucl.ac.uk     23/01/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is used to test the accuracy of a least-squares regression
% fit of linear parameters for the modelling of the Kalman ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Load the data to be fitted:
load('tmp_rov_sphere.mat'); %'tmp.mat' %'tmp1.mat'
dt = t(2)-t(1);
tEnd = t(end);

%% Prepare the data for system identification:
% Keep only 4 DOF:
x_4dof = [x(:,1:3),x(:,6:9),x(:,12)];
f_4dof = [f(:,1:3),f(:,6)];
simin = [t,f_4dof,x_4dof];

%% Load the ROV data:
% load('rov.mat');

%% Generate the LTI model of the Kaxan ROV in 4 DOF:
% M = [rov.M_B(1:3,1:3),rov.M_B(1:3,6);rov.M_B(6,1:3),rov.M_B(6,6)] + ...
%     [rov.M_A(1:3,1:3),rov.M_A(1:3,6);rov.M_A(6,1:3),rov.M_A(6,6)];
% D = [rov.D_l(1:3,1:3),rov.D_l(1:3,6);rov.D_l(6,1:3),rov.D_l(6,6)];
% G = zeros(4);
% M_inv = pinv(M);
% 
% A = [zeros(4),eye(4);-M_inv*G,-M_inv*D];
% B = [zeros(4);M_inv];
% E = [zeros(4);M_inv];
% C = eye(8);

% % Load pre-generated identified system:
% load('ss_rov.mat');
% sys = ss(A,B,C,0);
% % Convert it to discrete time:
% sysd = c2d(sys,0.1);

% Specify values for the covariance matrix:
R = 0.1*eye(12);

%% Run the Simulink file:
% Simulink file:
sfile = 'rls_est';
% Load the Simulink file:
load_system(sfile);

% Run the Simulink file:
sout = sim(sfile,'StopTime',num2str(tEnd));

% Close the Simulink file:
close_system(sfile);

%% Post-processing:
% Extract the data to be plotted:
x_hat = sout.get('logsout').getElement('x_hat').Values.Data;
Ad = sout.get('logsout').getElement('A').Values.Data;
Bd = sout.get('logsout').getElement('B').Values.Data;

% Plot the data:
sysid_plot(t,x_4dof,x_hat);
% % Plot the matrix entries:
% mat_plot(Ad,Bd,sysd.A,sysd.B);