global agentType;
simulations = {};
i = 1;
% NN optimization
simulations{i}          = struct();
simulations{i}.popSize  = 35;
simulations{i}.type     = 'simpleNN';           % 0.0024908491
simulations{i}.nnSize   = 15;
simulations{i}.LB       = -1 * ones(1,1 + 3*simulations{i}.nnSize + 1);
simulations{i}.UB       =  1 * ones(1,1 + 3*simulations{i}.nnSize + 1);
i = i + 1;
% Pinciroli optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'pinciroli';          % 0.0000121633
simulations{i}.LB       = zeros(1,2);
simulations{i}.UB       = 1 * ones(1,2);
i = i + 1;
% Polynomial optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'polynomial';
simulations{i}.LB       = [0 -5*ones(1,13)];
simulations{i}.UB       = [1  5*ones(1,13)];
i = i + 1;
% Sinusoid optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'sinusoid';
simulations{i}.LB       = [0 -1  -15  0 0  -15  0 0  -15  0 0  -15  0 0];
simulations{i}.UB       = [1  1   15 50 1   15 50 1   15 50 1   15 50 1];
i = i + 1;
% Set general simulation parameters
simPar = struct(...
    'type',                 '',...
    'simTime',              15, ...
    'trialSize',            3, ...
    'fps',                  15, ...
    'nAgents',              0, ...  % Pinciroli agents
    'polyAgents',           0, ...  % Polynomial agents
    'nnAgents',             0, ...  % Simple neural network agets
    'sinusoidAgents',       0, ...  % Sinusoid agents
    'circle_radius',        3, ...
    'seperation_range',     1.5, ...
    'init',                 'random', ...
    'size',                 [7 7], ...
    'v_max',                2, ...
    'distance_cost',        1, ...
    'velocity_cost',        2, ...
    'collision_cost',       3, ...
    'nnSize',               15 ...
    );
simPar.mission = {'cyberzooBucket'};
%% Run all simulations
for si = 1:length(simulations)
    switch simulations{si}.type 
        case 'pinciroli'
            simPar.type             = 'pinciroli';
            simPar.nAgents          = 9;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.12 0.12];
        case 'polynomial'
            simPar.type             = 'polynomial';
            simPar.nAgents          = 9;
            simPar.polyAgents       = 9;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 0 0 0 0 0 0 -0.12 0 0 0 0 0 0.12];
        case 'sinusoid'
            simPar.type             = 'sinusoid';
            simPar.nAgents          = 9;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 9;
            sampleGenome            = [0.1 -0.0267 -5.4729 0.1886 0.7875 0.9981 34.0776 0.6417 1.3918 37.7451 0.9187 1.4766 15.6458 0.9927];
        case 'simpleNN'
            simPar.type             = 'simpleNN';
            simPar.nnSize           = simulations{si}.nnSize;
            simPar.nAgents          = 9;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 9;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 rand(1,3*simPar.nnSize+1)];
            simPar.net              = feedforwardnet([simPar.nnSize]);
            %simPar.net.layers{1}.transferFcn = 'logsig';
            %simPar.net.layers{2}.transferFcn = 'radbas';
            %simPar.net.layers{3}.transferFcn = 'purelin';
            simPar.net              = configure(simPar.net, [(-simPar.seperation_range - 0.3)/4 (4 -simPar.seperation_range - 0.3)/4], [-1 1]);
    end
    %% Do a sample simulation to get an estimate of the time
    pT              = tic;
    [cost]          = sim_calc_cost(simPar, sampleGenome, false);
    simt            = toc(pT);
    fprintf(strcat(['\n\nSimulation ' simPar.type ' took ' num2str(simt) 's - performance: (cost: ' num2str(cost) ')\n\n']));
    %% Run GA
    agentType = simPar.type;
    options = optimoptions('ga',...
        'PopulationSize',simulations{si}.popSize, ...
        'Display','iter', ...
        'PlotFcn',{@gaplotbestf_scale @(options,state,flag) gaPlotAgentFunction(simPar, options, state, flag)},...
        'UseParallel',1,...
...%        'OutputFcn',@outputfun_ga,...
        'CrossoverFraction',0.8...
     );
    [x,fval,exitflag,output,population,scores] = ga(@(x) sim_calc_cost(simPar, x, false), length(simulations{si}.LB),[],[],[],[],simulations{si}.LB,simulations{si}.UB,[],options);
    %% save genome to file
    save(strcat(['ga-' simulations{si}.type '-' num2str(simPar.simTime) 's-' num2str(scores(1)) '.mat']),'x','fval','exitflag','output','population','scores');
    %% Genetic optimization finished, sim the winner and make video
    sim_calc_cost(simPar, x, true);
end
