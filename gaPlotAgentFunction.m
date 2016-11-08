function state = gaPlotAgentFunction(simPar, ~,state,flag)
%GAPLOTBESTINDIV Plots the best individual.
%   STATE = GAPLOTBESTINDIV(OPTIONS,STATE,FLAG) plots the best 
%   individual's genome as a histogram, with the number of bins
%   in the histogram equal to the length of the genome.
%
%   Example:
%    Create an options structure that uses GAPLOTBESTINDIV
%    as the plot function
%     options = optimoptions('ga','PlotFcn',@gaplotbestindiv);

%   Copyright 2003-2015 The MathWorks, Inc.
global agentType;
if  size(state.Score,2) > 1
    title('Best Individual Plot: not available','interp','none');
    return;
end

switch(agentType)
    case 'pinciroli'
        tmp_agent = PinciroliAgent(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'polynomial'
        tmp_agent = polyAgent(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'sinusoid'
        tmp_agent = sinusoidAgent(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'simpleNN'
        tmp_agent      = simpleNNAgent(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
        tmp_agent.net  = feedforwardnet([5]);
        tmp_agent.net  = configure(tmp_agent.net, [(-tmp_agent.seperation_range - 0.3)/4 (4 -tmp_agent.seperation_range - 0.3)/4], [-1 1]);
        tmp_agent.net  = setwb(tmp_agent.net,genome(2:end));
end
[~,i] = min(state.Score);
tmp_agent.genome = state.Population(i,:);
tmp_agent.seperation_range = simPar.seperation_range;
x = 0:0.05:tmp_agent.cam_range;
switch flag
    case 'init'
        y = tmp_agent.getAgentFunction(x);
        h = plot(x,y);
        set(h,'Tag','gaPlotAgentFunction');
        hold on;
        plot([x(1) x(end)],[simPar.v_max simPar.v_max],'--','Color','black');
        hold off;
        grid on;
        set(gca,'xlim',[0, tmp_agent.cam_range])
        set(gca,'ylim',[min(y), 2 * simPar.v_max])
        title('Current Best Individual','interp','none')
        xlabel('Distance [m]','interp','none');
        ylabel('Velocity response [m/s]','interp','none');
    case 'iter'
        h = findobj(get(gca,'Children'),'Tag','gaPlotAgentFunction');
        set(h,'Ydata',tmp_agent.getAgentFunction(x));
end
