% uuvSimRun.m     e.anderlini@ucl.ac.uk     23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with model predictive control.
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
T = [rov.T(1:3,:);rov.T(6,:)]; % thrust allocation matrix for 4 DOF
Tinv = pinv(T);                % inverse of the thrust allocation matrix

%% Generate the LTI model of the Kaxan ROV in 4 DOF:
% M = [rov.M_B(1:3,1:3),rov.M_B(1:3,6);rov.M_B(6,1:3),rov.M_B(6,6)] + ...
%     [rov.M_A(1:3,1:3),rov.M_A(1:3,6);rov.M_A(6,1:3),rov.M_A(6,6)];
% D = [rov.D_l(1:3,1:3),rov.D_l(1:3,6);rov.D_l(6,1:3),rov.D_l(6,6)];
% G = zeros(4);
% M_inv = pinv(M);
% 
% % Create the state-space matrices:
% A = [zeros(4),eye(4);-M_inv*G,-M_inv*D];
% B = [zeros(4);M_inv];
% E = [zeros(4);M_inv];
% C = eye(8);

% Load the identified system as an alternative:
load('ss_rov.mat');
% Create the continuous-timestate-space system:
sys  = ss(A,B,C,0);

%% MPC:
% Define the prediction and control horizons:
p = 25;
m = 10;
% Define the time step of the model predictive control:
dt = 0.1;
% Define other parameters:
nu = 4;   % no. manipulated variables (4 DOF thrust vector)
W.MV     = zeros(1,nu);       % manipulated variables weights
W.MVRate = 0.1*ones(1,nu);    % manipulated variables increment weights
W.OV     = [1,1,1,1,0,0,0,0]; % output variables weights

% Initialize the model predictive control object:
mpc_kaxan = mpc(sys,dt,p,m,W);

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_mpc';
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