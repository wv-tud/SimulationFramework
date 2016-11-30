function state = gaPlotResults(simPar, options,state,flag)
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
        fakeNet             = struct();
        fakeNet.numLayers   = length(simPar.nnSize);
        fakeNet.IW          = state.Population(i,simPar.net.i_IW+1);
        fakeNet.LW          = state.Population(i,simPar.net.i_LW+1);
        fakeNet.IB          = state.Population(i,simPar.net.i_IB+1);
        fakeNet.OB          = state.Population(i,simPar.net.i_OB+1);
        tmp_agent           = Agent_simpleNN(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
        tmp_agent.net       = fakeNet;
        tmp_agent.v_max     = simPar.v_max;
end
tmp_agent.genome            = state.Population(i,:);
tmp_agent.seperation_range  = simPar.seperation_range;
x                           = 0:0.05:tmp_agent.cam_range;
switch flag
    case 'init'
        %% Plot field
        subplot(2,2,1);
        hold on;
        simPar.type             = 'pinciroli';
        simPar.t                = 1/simPar.fps;
        [simPar.nnAgents,simPar.polyAgents,simPar.sinusoidAgents] = deal(0);
        [~,sArena]              = sim_calc_cost(simPar,[0.12 0.12], false);
        agentIndices            = sArena.chunkSplit(1:sArena.nAgents,length(simPar.field));
        for i=1:length(simPar.field)
            sArena.agents{agentIndices(i,1)}.plotGlobalAttraction(-simPar.size(1)/2:0.1:simPar.size(1)/2,-simPar.size(2)/2:0.1:simPar.size(2)/2, false);
        end
        %% Plot best score
        H = subplot(2,2,2);
        hold on;
        set(gca,'xlim',[0,options.MaxGenerations+1]);
        xlabel('Generation','interp','none');
        ylabel('Fitness value','interp','none');
        plotBest = plot(state.Generation,min(state.Score),'.k');
        set(plotBest,'Tag','gaplotbestf');
        plotMean = plot(state.Generation,meanf(state.Score),'.b');
        set(plotMean,'Tag','gaplotmean');
        title(['Best: ',' Mean: '],'interp','none');
        grid minor;
        hold on;
        %% Plot agent function
        subplot(2,2,[3 4]);
        hold on;
        plot([x(1) x(end)],[simPar.v_max simPar.v_max],'--','Color','black');
        plot([x(1) x(end)],[-simPar.v_max -simPar.v_max],'--','Color','black');
        sigma = tmp_agent.seperation_range+tmp_agent.collision_range;
        plot([sigma sigma],[-simPar.v_max simPar.v_max],'--','Color','black');
        plot([x(1) x(end)],[0 0],'-','Color','black');
        y = tmp_agent.getAgentFunction(x);
        h = plot(x,y);
        set(h,'Tag','gaPlotAgentFunction');
        grid minor;
        set(gca,'xlim',[0, tmp_agent.cam_range])
        set(gca,'ylim',[-1.5 * simPar.v_max, 1.5 * simPar.v_max])
        title('Current Best Individual','interp','none')
        xlabel('Distance [m]','interp','none');
        ylabel('Velocity response [m/s]','interp','none');
    case 'iter'
        %% Plot best score
        subplot(2,2,2);
        best        = min(state.Score);
        m           = meanf(state.Score);
        plotBest    = findobj(get(gca,'Children'),'Tag','gaplotbestf');
        plotMean    = findobj(get(gca,'Children'),'Tag','gaplotmean');
        newX        = [get(plotBest,'Xdata') state.Generation];
        newY        = [get(plotBest,'Ydata') best];
        bestCol     = newY;
        set(plotBest,'Xdata',newX, 'Ydata',newY);
        newY        = [get(plotMean,'Ydata') m];
        set(plotMean,'Xdata',newX, 'Ydata',newY);
        set(get(gca,'Title'),'String',sprintf('Best: %g Mean: %g',best,m));
        set(gca,'xlim',[0,state.Generation]);
        set(gca,'ylim',[max(0,min(best-0.1,best-std(bestCol))),max(best+0.1,best+std(bestCol))]);
        %% Plot agent function
        subplot(2,2,[3 4]);
        h = findobj(get(gca,'Children'),'Tag','gaPlotAgentFunction');
        set(h,'Ydata',tmp_agent.getAgentFunction(x));
    case 'done'
        hold off
        %% Plot best score
        LegnD = legend('Best fitness','Mean fitness');
        set(LegnD,'FontSize',8);
end
end

%------------------------------------------------
function m = meanf(x)
nans = isnan(x);
x(nans) = 0;
n = sum(~nans);
n(n==0) = NaN; % prevent divideByZero warnings
% Sum up non-NaNs, and divide by the number of non-NaNs.
m = sum(x) ./ n;
end