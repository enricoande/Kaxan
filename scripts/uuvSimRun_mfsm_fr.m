% uuvSimRun_mfsm_fr.m     e.anderlini@ucl.ac.uk     09/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with model-free sliding-modes control. The file relies on fast restart to
% simulate the ROV picking up an object.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Run the set-up file:
rovSimSetup;
tEnd1 = 35;

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

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_mfsm';
% Load the Simulink file:
load_system(sfile);
% Setup Fast Restart:
% set_param(sfile,'FastRestart','on');
% set_param(sfile,'SaveFinalState','on');
% set_param(sfile,'SaveCompleteFinalSimState','on');
% hws = get_param(sfile,'modelworkspace');
% hws.assignin('rov', rov);
% save_system(sfile);

%% Run the first part of the simulation with the Kaxan ROV:
sout = sim(sfile,'StopTime',num2str(tEnd1));

% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('state').Values.Data;
f = [sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
x_des = [sout.get('logsout').getElement('des_pos').Values.Data,...
    sout.get('logsout').getElement('des_vel').Values.Data];

% % Snatch the final simstate:
% assignin('base','xFinal',sout.get('xFinal'));

%% Run the second part of the simulation with the ROV carrying the sphere:
% save_system(sfile);
close_system(sfile);
% load_system(sfile);
clear rov;
load('rov_sphere.mat');
ics = x(end,:);
load_system(sfile);
% hws = get_param(sfile,'modelworkspace');
% hws.assignin('rov', rov);
% save_system(sfile);
% set_param(sfile,'SimulationCommand','update');
% set_param(bdroot,'SimulationCommand','update');
% set_param(sfile,'LoadInitialState','on');
% set_param(sfile,'InitialState','xFinal');
% hws = get_param(sfile,'modelworkspace');
% hws.assignin('rov', rov);
% assignin('base','rov',rov);
% set_param(sfile,'SimulationCommand','update');
sout = sim(sfile,'StopTime',num2str(tEnd1));

%% Close the Simulink file:
% set_param(sfile,'FastRestart','off');
% set_param(sfile,'LoadInitialState','off');
% set_param(sfile,'SaveFinalState','off');
% set_param(sfile,'SaveCompleteFinalSimState','off');
% save_system(sfile);
close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = [t;sout.tout+tEnd1];
x = [x;sout.get('logsout').getElement('state').Values.Data];
f = [f;sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
x_des = [x_des;sout.get('logsout').getElement('des_pos').Values.Data,...
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