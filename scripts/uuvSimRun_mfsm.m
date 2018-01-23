% uuvSimRun_mfsm.m     e.anderlini@ucl.ac.uk     22/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with model-free sliding-modes control.
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

%% Gain matrices:
A  = eye(4);
Kd = 500*eye(4);
Ki = 0.1*eye(4);

%% Waypoints and trajectory initialization:
waypoints = [0,0,0,0,0,0;...
             2,1,1,0,0,1]; %...
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
sfile = 'uuvSim_mfsm';
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
x_des = [sout.get('logsout').getElement('des_pos').Values.Data,...
    sout.get('logsout').getElement('des_vel').Values.Data];

% Plot the AUV's motions:
plotMotions(t,x);
% Plot the desired motions:
plotMotions(t,x_des);
% Plot the difference in motions (error):
plotMotions(t,x_des-x);

% % Plot the AUV's forces:
% plotForces(t,f);
% % Plot the AUV's path:
% plotPath(x,waypoints);
% % Animate the AUV's motion:
% animateAUV(t,x,50,1,8);