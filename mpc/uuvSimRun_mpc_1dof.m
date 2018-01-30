% uuvSimRun.m     e.anderlini@ucl.ac.uk     30/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with model predictive control in surge.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Run the set-up file:
rovSimSetup;

% Initial conditions:
ics = zeros(12,1);             % initial conditions (m & rad)
v_c = [0;0;0;0;0;0];           % current velocity (m/s)

% Pre-processing:
T = rov.T(1,:);                % thrust allocation matrix for surge
Tinv = pinv(T);                % inverse of the thrust allocation matrix

%% Generate the LTI model of the Kaxan ROV in 4 DOF:
M = rov.M_B(1,1) + rov.M_A(1,1);
D = rov.D_l(1,1);
G = zeros(1);
M_inv = pinv(M);

% Create the state-space matrices:
A = [zeros(1),eye(1);-M_inv*G,-M_inv*D];
B = [zeros(1);M_inv];
E = [zeros(1);M_inv];
C = eye(2);

% Create the continuous-timestate-space system:
sys  = ss(A,B,C,0);

%% MPC:
% Define the prediction and control horizons:
p = 25;
m = 10;
% Define the time step of the model predictive control:
dt = 0.1;
% Initialize the model predictive control object:
mpc_kaxan_1dof = mpc(sys,dt,p,m);

%% Waypoints and trajectory initialization:
waypoints = [0,0,0,0,0,0;...
             2,2,2,0,0,1]; %...
%              2,2,0,0,0,0;...
%              2,2,2,0,0,0;...
%              2,2,2,0,0,2*pi;...
%              2,2,0,0,0,0;...
%              2,0,0,0,0,0;...
%              0,0,0,0,0,0;...
%              0,0,0,0,0,-2*pi;...
%              2,2,2,0,0,0;...
%              3,1,1,0,0,-pi;...
%              2,2,2,0,0,pi];
trj_type = 'minimum_snap';
% waypoints = [5,1,0.2];
% trj_type = 'helix';

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_mpc_1dof';
% Load the Simulink file:
load_system(sfile);

%% Run the first shot:
sout = sim(sfile,'StopTime',num2str(mdl.tEnd));

%% Close the Simulink file:
close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('state').Values.Data;
f = [sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
% x_des = [sout.get('logsout').getElement('des_pos').Values.Data,...
%     sout.get('logsout').getElement('des_vel').Values.Data];

% Plot the AUV's motions:
plotMotions(t,x);
% % Plot the desired motions:
% plotMotions(t,x_des);
% % Plot the difference in motions (error):
% plotMotions(t,x_des-x);

% % Plot the AUV's forces:
% plotForces(t,f);
% % Plot the AUV's path:
% plotPath(x,waypoints);
% % Animate the AUV's motion:
% animateAUV(t,x,50,1,8);