% uuvSimRun.m     e.anderlini@ucl.ac.uk     18/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with PID control.
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

%% PID controller gains:
kp = [500;500;500;100];        % proportional gain   %[500;500;500;100];
kd = [5;5;5;5];                % derivative gain
ki = [10;10;10;10];            % integral gain

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_pid';
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
% % Plot the desired motions:
% plotMotions(t,x_des);
% % Plot the difference in motions (error):
% plotMotions(t,x_des-x);

% Plot the AUV's forces:
% plotForces(t,f);
% % Plot the AUV's path:
% plotPath(x,waypoints);
% % Animate the AUV's motion:
% animateAUV(t,x,50,1,8);