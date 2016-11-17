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
[~,i] = min(state.Score);
switch(agentType)
    case 'pinciroli'
        tmp_agent = Agent_pinciroli(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'polynomial'
        tmp_agent = Agent_polynomial(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'sinusoid'
        tmp_agent = Agent_sinusoid(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
    case 'simpleNN'
        tmp_agent           = Agent_simpleNN(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
        tmp_net             = feedforwardnet([simPar.nnSize]);
        tmp_net             = configure(tmp_net, [(-tmp_agent.seperation_range - 0.3)/4 (4 -tmp_agent.seperation_range - 0.3)/4], [-1 1]);
        tmp_net             = setwb(tmp_net,state.Population(i,2:end));
        fakeNet             = {};
        fakeNet.numLayers   = tmp_net.numLayers;
        fakeNet.IW          = tmp_net.IW;
        fakeNet.LW          = tmp_net.LW;
        fakeNet.b           = tmp_net.b;
        tmp_agent.net       = fakeNet;
        tmp_agent.v_max     = simPar.v_max;
        resp                = tmp_agent.getAgentFunction(0:0.01:tmp_agent.cam_range);
        tmp_agent.a         = simPar.v_max / max(abs(resp));
end
tmp_agent.genome            = state.Population(i,:);
tmp_agent.seperation_range  = simPar.seperation_range;
x                           = 0:0.05:tmp_agent.cam_range;
switch flag
    case 'init'
        hold on;
        plot([x(1) x(end)],[simPar.v_max simPar.v_max],'--','Color','black');
        plot([x(1) x(end)],[-simPar.v_max -simPar.v_max],'--','Color','black');
        plot([x(1) x(end)],[0 0],'-','Color','black');
        y = tmp_agent.getAgentFunction(x);
        h = plot(x,y);
        set(h,'Tag','gaPlotAgentFunction');
        hold off;
        grid on;
        set(gca,'xlim',[0, tmp_agent.cam_range])
        set(gca,'ylim',[-1.5 * simPar.v_max, 1.5 * simPar.v_max])
        title('Current Best Individual','interp','none')
        xlabel('Distance [m]','interp','none');
        ylabel('Velocity response [m/s]','interp','none');
    case 'iter'
        h = findobj(get(gca,'Children'),'Tag','gaPlotAgentFunction');
        set(h,'Ydata',tmp_agent.getAgentFunction(x));
end
