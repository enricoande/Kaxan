% trj_creator.m     e.anderlini@ucl.ac.uk     06/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is used to generate a desired trajectory for later testing.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Initialization:
duration = 60;
dt = 0.001;
t = 0:dt:duration;
n = length(t); 
x  = zeros(12,n);

%% Generate the first segment of the trajectory:
waypoints = [0,0,0,0,0,0;2,2,2,0,0,1];

trj_minsnap  = Trajectory('minimum_snap',waypoints,10,dt);

for i=1:round(10/dt+1)
    % Minimum-snap trajectory:
    trj_minsnap = trj_minsnap.trajectory_generation(t(i));
    x(1:6,i) = trj_minsnap.des_disp;
    x(7:12,i) = trj_minsnap.des_vel;
end

%% Generate the second segment of the trajectory:
for i=round(10/dt+2):round(15/dt)
    x(1:6,i)  = [2,2,2,0,0,1];
    x(7:12,i) = [0,0,0,0,0,0];
end

%% Generate the third segment of the trajectory:
waypoints = [2,2,2,0,0,1;0,0,0,0,0,0];

trj_minsnap  = Trajectory('minimum_snap',waypoints,10,dt);

for i=1:round(10/dt+1)
    % Minimum-snap trajectory:
    trj_minsnap = trj_minsnap.trajectory_generation(t(i));
    x(1:6,i+round(15/dt)) = trj_minsnap.des_disp;
    x(7:12,i+round(15/dt)) = trj_minsnap.des_vel;
end

%% Generate the fourth segment of the trajectory:
for i=round(25/dt+2):round(35/dt)
    x(1:6,i)  = [0,0,0,0,0,0];
    x(7:12,i) = [0,0,0,0,0,0];
end

%% Generate the fifth segment of the trajectory:
waypoints = [0,0,0,0,0,0;2,2,2,0,0,1];

trj_minsnap  = Trajectory('minimum_snap',waypoints,10,dt);

for i=1:round(10/dt+1)
    % Minimum-snap trajectory:
    trj_minsnap = trj_minsnap.trajectory_generation(t(i));
    x(1:6,i+round(35/dt)) = trj_minsnap.des_disp;
    x(7:12,i+round(35/dt)) = trj_minsnap.des_vel;
end

%% Generate the sixth segment of the trajectory:
for i=round(45/dt+2):round(50/dt)
    x(1:6,i)  = [2,2,2,0,0,1];
    x(7:12,i) = [0,0,0,0,0,0];
end

%% Generate the seventh segment of the trajectory:
waypoints = [2,2,2,0,0,1;0,0,0,0,0,0];

trj_minsnap  = Trajectory('minimum_snap',waypoints,10,dt);

for i=1:round(10/dt+1)
    % Minimum-snap trajectory:
    trj_minsnap = trj_minsnap.trajectory_generation(t(i));
    x(1:6,i+round(50/dt)) = trj_minsnap.des_disp;
    x(7:12,i+round(50/dt)) = trj_minsnap.des_vel;
end

x  = x';
x(:,7:12) = x(:,7:12)*0.01/dt;
plotMotions(t,x);

%% Save the trajectory to file:
trj = [t',x];
save('../data/trj.mat','trj');