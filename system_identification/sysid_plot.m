function sysid_plot(t,x,x_hat)
% sysid_plot.m     e.anderlini@ucl.ac.uk     23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the translations and rotations in 4 DOF to
% check the quality of the fit of the identified system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Prepare the titles and labels of the plots:
ylab   = {'$x$ (m)','$y$ (m)','$z$ (m)','$\psi$ (rad)',...
    '$\dot{x}$ (m/s)','$\dot{y}$ (m)','$\dot{z}$ (m/s)',...
    '$\dot{\psi}$ (rad/s)'};
titles = {'Surge','Sway','Heave','Yaw'};

%% Produce the plots for the translational and rotational motions:
dof = length(titles);
for i=1:dof
    % Displacement:
    figure;
    subplot(2,1,1);
    plot(t,x(:,i),'--');
    hold on;
    plot(t,x_hat(:,i));
    hold off;
    ylabel(ylab{i},'Interpreter','Latex');
    l = legend('actual','estimated','Location','Best');
    set(l,'Interpreter','Latex');
    grid on;
    set(gca,'TickLabelInterpreter','Latex')
    set(gcf,'color','w');
    % Velocity:
    subplot(2,1,2);
    plot(t,x(:,i+dof),'--');
    hold on;
    plot(t,x_hat(:,i+dof));
    hold off;
    xlabel('Time (s)','Interpreter','Latex');
    ylabel(ylab{i+dof},'Interpreter','Latex');
    l = legend('actual','estimated','Location','Best');
    set(l,'Interpreter','Latex');
    grid on;
    set(gca,'TickLabelInterpreter','Latex')
    set(gcf,'color','w');
end

end