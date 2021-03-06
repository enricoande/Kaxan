% rovSimSetup.m      e.anderlini@ed.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the parameters required for the simulation of the
% ROV.
%
% This code has been adapted from the code by Gordon Parker at Michigan
% Technological University.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear;
% close all;

%% Simulation set-up:
mdl.tStep = 0.01; % time step length (s)
mdl.tEnd  = 70; %20;   % end time (s)

%% ROV model set-up:
load('rov.mat');  % rov object  %'rov.mat' %'rov_sphere.mat'

%% Trajectory data:
load('trj.mat');

% Disturbance data:
% d = zeros(length(trj),1);
d = [trj(1:101,1).^2;ones(length(trj)-101,1)];
d = [trj(:,1),d];

% % Trial trajectory:
% t = 0:0.001:70;
% trj = [t',zeros(length(t),12)];