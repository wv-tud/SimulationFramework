global agentType;
% Set general simulation parameters
simPar = struct(...
    'type',                 '',...
    'simTime',              20, ...
    'trialSize',            3, ...
    'fps',                  15, ...
    'nAgents',              0, ...  % Pinciroli agents
    'polyAgents',           0, ...  % Polynomial agents
    'nnAgents',             0, ...  % Simple neural network agets
    'sinusoidAgents',       0, ...  % Sinusoid agents
    'seperation_range',     1.75, ...
    'collision_range',      0.3, ...
    'camera_range',         4.0, ...
    'init',                 'random', ...
    'size',                 [15 15], ...
    'v_max',                1.4, ...
    'distance_cost',        1, ...
    'velocity_cost',        0.1, ...
    'collision_cost',       1e8, ...
    'nnSize',               [10 10], ...
    'boc',                  1 ...
    );
simPar.mission          = {'cyberzooBucket'};
simulations             = {};
i                       = 1;
% NN optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'simpleNN';
simulations{i}.nnSize   = 15;
switch(length(simulations{i}.nnSize))
    case 1
        simulations{i}.genomeNetLength = 3 * simulations{i}.nnSize + 1;
        IB = 1:simulations{i}.nnSize;
        OB = IB(end)+1;
        IW = OB(end)+1:OB(end)+simulations{i}.nnSize;
        LW = IW(end)+1:simulations{i}.genomeNetLength;
    case 2
        simulations{i}.genomeNetLength = simulations{i}.nnSize(1) + simulations{i}.nnSize(1) * simulations{i}.nnSize(2) + simulations{i}.nnSize(2) + simulations{i}.nnSize(1) + simulations{i}.nnSize(2) + 1;
        IB = 1:(simulations{i}.nnSize(1) + simulations{i}.nnSize(2));
        OB = IB(end)+1;
        IW = OB(end)+1:(OB(end) + simulations{i}.nnSize(1) + simulations{i}.nnSize(1) * simulations{i}.nnSize(2));
        LW = IW(end)+1:simulations{i}.genomeNetLength;
end
%simulations{i}.LB      = -15 / (simulations{i}.nnSize(end)) * ones(1,1 + simulations{i}.genomeNetLength);
%simulations{i}.UB      =  15 / (simulations{i}.nnSize(end)) * ones(1,1 + simulations{i}.genomeNetLength);
simulations{i}.LB       = [0  -1 * ones(1, simulations{i}.genomeNetLength)];
simulations{i}.UB       = [0.5 1 * ones(1, simulations{i}.genomeNetLength)];
simulations{i}.LB(IB+1) = -3;
simulations{i}.UB(IB+1) =  3;
simulations{i}.LB(OB+1) = -4;
simulations{i}.UB(OB+1) =  4;
i = i + 1;
% Pinciroli optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'pinciroli';          
simulations{i}.LB       = zeros(1,2);
simulations{i}.UB       = [0.5 0.5];
i = i + 1;
% Polynomial optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'polynomial';
simulations{i}.LB       = [0 -5*ones(1,13)];
simulations{i}.UB       = [0.5  5*ones(1,13)];
i = i + 1;
% Sinusoid optimization
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'sinusoid';
simulations{i}.LB       = [0   -0.5  -15  0 0  -15  0 0  -15  0 0  -15  0 0];
simulations{i}.UB       = [0.5  0.5   15  1 1   15 50 1   15  1 1   15  1 1];
i = i + 1;
%% Run all simulations
x_store         = cell(length(simulations),1);
simPar_store    = cell(length(simulations),1);
for si = 1:length(simulations)
    switch simulations{si}.type 
        case 'pinciroli'
            simPar.type             = 'pinciroli';
            simPar.nAgents          = 20;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 0.01];
        case 'polynomial'
            simPar.type             = 'polynomial';
            simPar.polyAgents       = 20;
            simPar.nAgents          = simPar.polyAgents;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 0 0 0 0 0 0 -0.12 0 0 0 0 0 0.12];
        case 'sinusoid'
            simPar.type             = 'sinusoid';
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 20;
            simPar.nAgents          = simPar.sinusoidAgents;
            sampleGenome            = [0.1 -0.0267 -5.4729 0.1886 0.7875 0.9981 34.0776 0.6417 1.3918 37.7451 0.9187 1.4766 15.6458 0.9927];
        case 'simpleNN'
            simPar.type             = 'simpleNN';
            simPar.nnSize           = simulations{si}.nnSize;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 20;
            simPar.sinusoidAgents   = 0;
            simPar.nAgents          = simPar.nnAgents;
            sampleGenome            = [0.1 rand(1,simulations{si}.genomeNetLength)];
            simPar.net              = struct();
            simPar.net.i_IB         = IB;
            simPar.net.i_OB         = OB;
            simPar.net.i_IW         = IW;
            simPar.net.i_LW         = LW;
    end
    %% Check lattice ratio (also checked inside sim_calc_cost but throws warnings)
    if simPar.camera_range/(simPar.seperation_range + simPar.collision_range)>=2
        simPar.camera_range = 1.95 * (simPar.seperation_range + simPar.collision_range);
        fprintf('WARNING: lattice ratio >= 2, limiting camera range to %.2fm\n', simPar.camera_range);
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
...%        'FitnessScalingFcn', {@fitscalingprop}, ...
        'Display','iter', ...
        'PlotFcn',{@gaPlotAgentScore @(options,state,flag) gaPlotAgentFunction(simPar, options, state, flag)},...
        'UseParallel',1,...
...%        'OutputFcn',@gaOutputFun,...
        'CrossoverFraction',0.8...
     );
    [x,fval,exitflag,output,population,scores] = ga(@(x) sim_calc_cost(simPar, x, false), length(simulations{si}.LB),[],[],[],[],simulations{si}.LB,simulations{si}.UB,[],options);
    %% save genome to file
    save(strcat(['./data/ga-' simulations{si}.type '-' num2str(simPar.simTime) 's-' num2str(scores(1)) '.mat']),'x','fval','exitflag','output','population','scores');
    %% Genetic optimization finished, sim the winner and make video
    x_store{si}         = x;
    simPar_store{si}    = simPar;
end
%% Create videos
uArena_store        = cell(length(simulations),1);
costStruct_store    = cell(length(simulations),1);
for i = 1:length(simulations)
    close all;
    agentType = simPar_store{i}.type;
    [~,costStruct_store{i}] = sim_calc_cost(simPar_store{i}, x_store{i}, true);
end
