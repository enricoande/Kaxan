% uuvSimRun.m     e.anderlini@ucl.ac.uk     13/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control
% with model predictive control. The file relies on fast restart to
% simulate the ROV picking up an object.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Run the set-up file:
rovSimSetup;
tEnd1 = mdl.tEnd/2;

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
% % Create the continuous-timestate-space system:
% sys  = ss(A,B,C,0);

%% MPC:
% Define variable names:
sys.InputName = {'T1','T2','T3','T4','d'};
% Define the indices of the model variables:
sys.InputGroup.MV = [1,2,3,4];
% Define the index of the disturbance:
sys.InputGroup.MD = 5;

% Define the prediction and control horizons:
p = 25;
m = 10;
% Define the time step of the model predictive control:
dt = 0.1;
% Define other parameters:
nu = 4;   % no. manipulated variables (4 DOF thrust vector) + noise
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

%% Run the first part of the simulation with the Kaxan ROV:
sout = sim(sfile,'StopTime',num2str(tEnd1-mdl.tStep));

% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('state').Values.Data;
f = [sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
tf = sout.get('logsout').getElement('control').Values.Data;
V = sout.get('logsout').getElement('V').Values.Data;

close_system(sfile);

%% Run the second part of the simulation with the ROV carrying the sphere:
clear rov mpc_kaxan;
% Load new simulation data:
load('rov_sphere.mat');
ics = x(end,:);
% Load the new MPC controller:
load('ss_rov_sphere.mat');
% Define variable names:
sys.InputName = {'T1','T2','T3','T4','d'};
% Define model variables:
sys.InputGroup.MV = [1,2,3,4];
% Define 
sys.InputGroup.MD = 5;
mpc_kaxan = mpc(sys,dt,p,m,W);
% Re-load the Simulink file:
load_system(sfile);
% Re-run Simulink:
sout = sim(sfile,'StopTime',num2str(tEnd1));

close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = [t;sout.tout+tEnd1];
x = [x;sout.get('logsout').getElement('state').Values.Data];
f = [f;sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
tf = [tf;sout.get('logsout').getElement('control').Values.Data];
V = [V;sout.get('logsout').getElement('V').Values.Data];

% Clip the thruster force:
ll_520 = rov.coeffs(1,1)*(-5)^3+rov.coeffs(1,2)*(-5)^2+...
    rov.coeffs(1,3)*(-5)+rov.coeffs(1,4);
ul_520 = rov.coeffs(1,1)*5^3+rov.coeffs(1,2)*5^2+...
    rov.coeffs(1,3)*5+rov.coeffs(1,4);
ll_540 = rov.coeffs(2,1)*(-5)^3+rov.coeffs(2,2)*(-5)^2+...
    rov.coeffs(2,3)*(-5)+rov.coeffs(2,4);
ul_540 = rov.coeffs(2,1)*5^3+rov.coeffs(2,2)*5^2+...
    rov.coeffs(2,3)*5+rov.coeffs(1,4);
for i=1:length(tf)
    tf(i,1) = min(tf(i,1),ul_520);
    tf(i,1) = max(tf(i,1),ll_520);
    tf(i,2) = min(tf(i,2),ul_520);
    tf(i,2) = max(tf(i,2),ll_520);
    tf(i,3) = min(tf(i,3),ul_540);
    tf(i,3) = max(tf(i,3),ll_540);
    tf(i,4) = min(tf(i,4),ul_540);
    tf(i,4) = max(tf(i,4),ll_540);
end

% Plot the AUV's motions:
plotMotions(t,x);

% % Plot the AUV's forces:
% plotForces(t,f);
% Plot the force in each thruster:
plotThrustersForces(t,tf);
% Plot the voltage in each thruster:
% plotThrustersVoltage(t,V);

% % Plot the AUV's path:
% plotPath(x,waypoints);
% % Animate the AUV's motion:
% animateAUV(t,x,50,1,8);