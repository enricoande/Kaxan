% sys_id.m     e.anderlini@ucl.ac.uk     23/01/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is used to identify a linear model of the Kaxan ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Load the data to be fitted:
load('tmp.mat');

%% Prepare the data for system identification:
% Keep only 4 DOF:
x_4dof = [x(:,1:3),x(:,6:9),x(:,12)];
f_4dof = [f(:,1:3),f(:,6:9),f(:,12)];

