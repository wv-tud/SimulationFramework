function state = gaPlotResults(simPar, options,state,flag, optimType)
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
switch optimType
    case 'ga'
        if  size(state.Score,2) > 1
            title('Best Individual Plot: not available','interp','none');
            return;
        end
        [~,i] = min(state.Score);
        genome = state.Population(i,:);
        curScore        = state.Score;
        curGen          = state.Generation;
        curMeanScore    = meanf(curScore);
        curBestScore    = min(curScore);
    case 'fmincon'
        genome          = state;
        curBestScore    = options.fval;
        curGen          = options.iteration;
end

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
        fakeNet.IW          = genome(simPar.net.i_IW+2)';
        fakeNet.IB          = genome(simPar.net.i_IB+2)';
        fakeNet.LW          = genome(simPar.net.i_LW+2);
        fakeNet.OB          = genome(simPar.net.i_OB+2);
        tmp_agent           = Agent_simpleNN(Mission(simPar.mission{1}),0,[0 0 0],[0 0]);
        tmp_agent.net       = fakeNet;
        tmp_agent.v_max     = simPar.v_max;
end
tmp_agent.genome            = genome;
tmp_agent.seperation_range  = simPar.seperation_range;
tmp_agent.cam_range         = simPar.camera_range;
x                           = 0:0.05:tmp_agent.cam_range;
switch flag
    case 'init'
        %% Plot field
        subplot(2,2,1);
        hold on;
        simPar.type             = 'pinciroli';
        simPar.simTime          = 1/simPar.fps;
        [simPar.nnAgents,simPar.polyAgents,simPar.sinusoidAgents] = deal(0);
        [~,sArena]              = sim_calc_cost(simPar,[0.12 0 0.12], false);
        agentIndices            = sArena.chunkSplit(1:sArena.nAgents,length(simPar.field));
        margin                  = 1.5;
        reso                    = size(1) / 10;
        l = 1;
        sArena.agents{agentIndices(l,1)}.plotGlobalAttraction(-margin*simPar.size(1)/2:reso:margin*simPar.size(1)/2,-margin*simPar.size(2)/2:reso:margin*simPar.size(2)/2, [0 0 10],false);
        %% Plot best score
        H = subplot(2,2,2);
        hold on;
        xlabel('Generation','interp','none');
        ylabel('Fitness value','interp','none');
        plotBest = plot(curGen,curBestScore,'.k');
        set(plotBest,'Tag','gaplotbestf');
        if strcmp(optimType,'ga')
            plotMean = plot(curGen,curMeanScore,'.b');
            set(plotMean,'Tag','gaplotmean');
            title(['Best: ',' Mean: '],'interp','none');
        else
            title('Best: ','interp','none');
        end
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
        h2 = plot(x,tmp_agent.loglo_int(abs(y)).*simPar.v_max,'--','Color','red');
        set(h2,'Tag','gaPlotAgentFunctionLoGlo');
        grid minor;
        set(gca,'xlim',[0, tmp_agent.cam_range])
        set(gca,'ylim',[-1.5 * simPar.v_max, 1.5 * simPar.v_max])
        title('Current Best Individual','interp','none')
        xlabel('Distance [m]','interp','none');
        ylabel('Velocity response [m/s]','interp','none');
    case 'iter'
        %% Plot best score
        subplot(2,2,2);
        hold on;
        plotBest    = findobj(get(gca,'Children'),'Tag','gaplotbestf');
        newX        = [get(plotBest,'Xdata') curGen];
        newY        = [get(plotBest,'Ydata') curBestScore];
        bestCol     = newY;
        set(plotBest,'Xdata',newX, 'Ydata',newY);
        if strcmp(optimType,'ga')
            plotMean    = findobj(get(gca,'Children'),'Tag','gaplotmean');
            newY        = [get(plotMean,'Ydata') curMeanScore];
            set(plotMean,'Xdata',newX, 'Ydata',newY);
            set(get(gca,'Title'),'String',sprintf('Best: %g Mean: %g',curBestScore,curMeanScore));
        else
            set(get(gca,'Title'),'String',sprintf('Best: %g',curBestScore));
        end
        set(gca,'xlim',[0,max(1,curGen)]);
        set(gca,'ylim',[max(0,min(curBestScore-0.1,curBestScore-std(bestCol))),max(curBestScore+0.1,curBestScore+std(bestCol))]);
        %% Plot agent function
        subplot(2,2,[3 4]);
        hold on;
        y = tmp_agent.getAgentFunction(x);
        h = findobj(get(gca,'Children'),'Tag','gaPlotAgentFunction');
        set(h,'Ydata',y);
        h2 = findobj(get(gca,'Children'),'Tag','gaPlotAgentFunctionLoGlo');
        set(h2,'Ydata',tmp_agent.loglo_int(abs(y)).*simPar.v_max);
    case 'done'
        hold off
        %% Plot best score
        if strcmp(optimType,'ga')
            LegnD = legend('Best fitness','Mean fitness');
            set(LegnD,'FontSize',8);
        end
end
if strcmp(optimType,'fmincon')
    state = false;
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