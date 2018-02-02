function mat_plot(Ad,Bd,Ad_actual,Bd_actual)
% mat_plot.m     e.anderlini@ucl.ac.uk     02/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the individual entries of the matrices of
% the discrete state-space system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~,~,n] = size(Ad);

%% Ad matrix:
for i=1:8
    figure;
    for j=1:8
        subplot(4,2,j);
        plot(squeeze(Ad(i,j,:)));
        hold on;
        plot(ones([1,n])*Ad_actual(i,j));
        hold off;
        ylabel(['$A_{',num2str(i),',',num2str(j),'}$'],...
            'Interpreter','Latex');
        grid on;
        set(gca,'TickLabelInterpreter','Latex')
        set(gcf,'color','w');
    end
end

%% Bd matrix:
for i=1:8
    figure;
    for j=1:4
        subplot(4,1,j);
        plot(squeeze(Bd(i,j,:)));
        hold on;
        plot(ones([1,n])*Bd_actual(i,j));
        hold off;
        ylabel(['$B_{',num2str(i),',',num2str(j),'}$'],...
            'Interpreter','Latex');
        grid on;
        set(gca,'TickLabelInterpreter','Latex')
        set(gcf,'color','w');
    end
end

end