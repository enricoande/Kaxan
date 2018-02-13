function plotThrustersForces(t,f)
% plotThrustersForces.m     e.anderlini@ucl.ac.uk     13/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the thrust force in each thruster of the 
% ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Forces:
figure;
plot(t,f(:,1),'--');
hold on;
plot(t,f(:,2),'-.');
hold on;
plot(t,f(:,3),'-');
hold on;
plot(t,f(:,4),':');
hold off;
xlabel('$t$ (s)','Interpreter','Latex');
ylabel('$T$ (N)','Interpreter','Latex');
l = legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4',...
    'Location','Best');
set(l,'Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
set(gcf,'color','w');

end