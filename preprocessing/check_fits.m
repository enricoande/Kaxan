% check_fits.m     e.anderlini@ucl.ac.uk      21/01/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script checks the quality of the fits to the thrusters' curves.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Extract the data points:
dir_name = './Kaxan/';        % directory name
line_520 = csvread([dir_name,'520.csv']);
line_540 = csvread([dir_name,'540.csv']);

%% Load the fitted coefficients:
load('rov.mat');

%% Fit data:
% 520:
y_520_1 = -80:0.1:0;
x_520_1 = rov.inv_coeffs(1,1)*(-y_520_1).^rov.inv_coeffs(1,2)+...
    rov.inv_coeffs(1,3);
y_520_2 = 0.1:0.1:100;
x_520_2 = rov.inv_coeffs(2,1)*y_520_2.^rov.inv_coeffs(2,2)+...
    rov.inv_coeffs(2,3);

y_540_1 = -100:0.1:0;
x_540_1 = rov.inv_coeffs(3,1)*(-y_540_1).^rov.inv_coeffs(3,2)+...
    rov.inv_coeffs(3,3);
y_540_2 = 0.1:0.1:150;
x_540_2 = rov.inv_coeffs(4,1)*y_540_2.^rov.inv_coeffs(4,2)+...
    rov.inv_coeffs(4,3);

x_520 = [x_520_1,x_520_2];
y_520 = [y_520_1,y_520_2];
x_540 = [x_540_1,x_540_2];
y_540 = [y_540_1,y_540_2];

%% Plot data:
figure;
scatter(line_520(:,1),line_520(:,2));
hold on;
plot(x_520,y_520);
hold off;
xlabel('Voltage (V)','Interpreter','Latex');
ylabel('Thrust Force (N)','Interpreter','Latex');
title('Thruster 520','Interpreter','Latex');
% l = legend('actual','fitted','Location','Best');
% set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

figure;
scatter(line_540(:,1),line_540(:,2));
hold on;
plot(x_540,y_540);
hold off;
xlabel('Voltage (V)','Interpreter','Latex');
ylabel('Thrust Force (N)','Interpreter','Latex');
title('Thruster 540','Interpreter','Latex');
% l = legend('actual','fitted','Location','Best');
% set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');