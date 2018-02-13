function plotThrustersVoltage(t,V)
% plotThrustersForces.m     e.anderlini@ucl.ac.uk     13/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the voltage in each thruster of the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Forces:
figure;
plot(t,V(:,1),'--');
hold on;
plot(t,V(:,2),'-.');
hold on;
plot(t,V(:,3),'-');
hold on;
plot(t,V(:,4),':');
hold off;
xlabel('$t$ (s)','Interpreter','Latex');
ylabel('$V$ (V)','Interpreter','Latex');
l = legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4',...
    'Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
set(gcf,'color','w');

end