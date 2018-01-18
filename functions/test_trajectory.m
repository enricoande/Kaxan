% test_trajectory.m     e.anderlini@ucl.ac.uk     27/11/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script tests the trajectory generator.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input data:
% Waypoints for hover, straight line and minimum-snap trajectory tests:
waypoints = [0,0,0,0,0,0;
             2,0,0,0,0,0;
             2,4,0,0,0,0;
             2,4,2,0,0,0];    % m
angles = [0,0,0,0,0,0;
          0,0,0,0,0,2*pi];    % rad
% Radius and height of helix trajectory:
radius = 1;             % m
height = 1;             % m
ang_speed = 0.5;        % rad/s
% Simulation duration:
duration = 10;          % s
% Time step:
dt = 0.01;              % s        

%% Initialize trajectories:
trj_hover    = Trajectory('hover',waypoints,duration,dt);
trj_line     = Trajectory('minimum_snap',waypoints(1:2,:),duration,dt);
trj_rotation = Trajectory('minimum_snap',angles,duration,dt);
trj_helix    = Trajectory('helix',[radius,height,ang_speed],duration,dt);
trj_minsnap  = Trajectory('minimum_snap',waypoints,duration,dt);

%% Initialize output data:
t = 0:dt:duration;      % time series
n = length(t);          % no. time steps
% Position vectors:
x_hover    = zeros(12,n);
x_line     = zeros(12,n);
x_rotation = zeros(12,n);
x_helix    = zeros(12,n);
x_minsnap  = zeros(12,n);

%% Obtain the desired trajectories:
for i=1:n
    % Hover trajectory:
    trj_hover = trj_hover.trajectory_generation(t(i));
    x_hover(1:6,i) = trj_hover.des_disp;
    x_hover(7:12,i) = trj_hover.des_vel;
    % Line trajectory:
    trj_line = trj_line.trajectory_generation(t(i));
    x_line(1:6,i) = trj_line.des_disp;
    x_line(7:12,i) = trj_line.des_vel;
    % Rotation trajectory:
    trj_rotation = trj_rotation.trajectory_generation(t(i));
    x_rotation(1:6,i) = trj_rotation.des_disp;
    x_rotation(7:12,i) = trj_rotation.des_vel;
    % Helix trajectory:
    trj_helix = trj_helix.trajectory_generation(t(i));
    x_helix(1:6,i) = trj_helix.des_disp;
    x_helix(7:12,i) = trj_helix.des_vel;
    % Minimum-snap trajectory:
    trj_minsnap = trj_minsnap.trajectory_generation(t(i));
    x_minsnap(1:6,i) = trj_minsnap.des_disp;
    x_minsnap(7:12,i) = trj_minsnap.des_vel;
end

%% Post-processing:
x_hover    = x_hover';
x_line     = x_line';
x_rotation = x_rotation';
x_helix    = x_helix';
x_minsnap  = x_minsnap';
% % Plot the desired motions of the trajectory:
% plotMotions(t,x_hover);
% plotMotions(t,x_line);
% plotMotions(t,x_rotation);
% plotMotions(t,x_helix);
plotMotions(t,x_minsnap);
% % Animate the desired AUV motions:
% animateAUV(t,x_hover,50,1,8);
% animateAUV(t,x_line,50,1,8);
% animateAUV(t,x_rotation,50,1,8);
% animateAUV(t,x_helix,50,1,8);
% animateAUV(t,x_minsnap,50,1,8);