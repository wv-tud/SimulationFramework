global agentType alt_gads noMouseActions;
lat_warning     = false;
v_max_warning   = false;
acc_sat_warning = false;
% Set general simulation parameters
simPar = struct(...
    'type',                 '',...
    'simTime',              60, ...
    'trialSize',            3, ...
    'fps',                  15, ...
    'nAgents',              0, ...  % Pinciroli agents
    'polyAgents',           0, ...  % Polynomial agents
    'nnAgents',             0, ...  % Simple neural network agets
    'sinusoidAgents',       0, ...  % Sinusoid agents
    'seperation_range',     1.75, ...
    'collision_range',      0.3, ...
    'camera_range',         3.5, ...
    'init',                 'random', ...
    'size',                 [10 10], ...
    'v_max',                3.5, ...
    'distance_cost',        10, ...
    'velocity_cost',        1, ...
    'collision_cost',       1e8, ...
    'seperation_cost',      100, ...
    'nnSize',               10, ...
    'moving_axes',          true,...
    'boc',                  1 ...
    );
simPar.velocity_cost    = simPar.velocity_cost / (2*simPar.v_max)^2;

load_global_fields;

simPar.mission          = {'cyberzooBucket'};
simulations             = {};
i                       = 1;
% NN optimization (two buck circ)
simulations{i}          = struct();
simulations{i}.popSize  = 75;
simulations{i}.type     = 'simpleNN';
simulations{i}.nnSize   = 25;
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
simulations{i}.LB       = [0     0 -1 * ones(1, simulations{i}.genomeNetLength)];
simulations{i}.UB       = [0.25  5  1 * ones(1, simulations{i}.genomeNetLength)];
simulations{i}.LB(IB+2) = -3;
simulations{i}.UB(IB+2) =  3;
simulations{i}.LB(OB+2) = -4;
simulations{i}.UB(OB+2) =  6;
simulations{i}.field    = two_point_circ;
i = i + 1;
% % NN optimization (two bucket circ)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = two_buck_circ;
% i = i + 1;
% % NN optimization (two circ)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = two_circ;
% i = i + 1;
% % NN optimization (xline)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = xline;
% i = i + 1;
% % NN optimization (bucket)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = buck;
% i = i + 1;
% 
% % Pinciroli optimization (two buck circ)
% simulations{i}          = struct();
% simulations{i}.popSize  = 75;
% simulations{i}.type     = 'pinciroli';
% simulations{i}.LB       = zeros(1,3);
% simulations{i}.UB       = [0.5 10 0.5];
% simulations{i}.field    = two_point_circ;
i = i + 1;
% % Pinciroli optimization (two circ)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = two_circ;
% i = i + 1;
% % Pinciroli optimization (xline)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = xline;
% i = i + 1;
% % Pinciroli optimization (bucket)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = buck;
% i = i + 1;
% 
% % Polynomial optimization (two buck circ)
% simulations{i}          = struct();
% simulations{i}.popSize  = 75;
% simulations{i}.type     = 'polynomial';
% simulations{i}.LB       = [0    0 -5*ones(1,13)];
% simulations{i}.UB       = [0.5 10  5*ones(1,13)];
% simulations{i}.field    = two_buck_circ;
% i = i + 1;
% % Polynomial optimization (two circ)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = two_circ;
% i = i + 1;
% % Polynomial optimization (xline)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = xline;
% i = i + 1;
% % Polynomial optimization (bucket)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = buck;
% i = i + 1;
% 
% % Sinusoid optimization (two buck circ)
% simulations{i}          = struct();
% simulations{i}.popSize  = 75;
% simulations{i}.type     = 'sinusoid';
% simulations{i}.LB       = [0    0 -0.5  -15  0 0  -15  0 0  -15  0 0  -15  0 0];
% simulations{i}.UB       = [0.5 10  0.5   15  1 1   15 50 1   15  1 1   15  1 1];
% simulations{i}.field    = two_buck_circ;
% i = i + 1;
% % Sinusoid optimization (two circ)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = two_circ;
% i = i + 1;
% % Sinusoid optimization (xline)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = xline;
% i = i + 1;
% % Sinusoid optimization (bucket)
% simulations{i}          = simulations{i-1};
% simulations{i}.field    = buck;
% i = i + 1;

%% Run all simulations
x_store         = cell(length(simulations),1);
simPar_store    = cell(length(simulations),1);
for si = 1:length(simulations)
    simPar.field = simulations{si}.field;
    switch simulations{si}.type
        case 'pinciroli'
            simPar.type             = 'pinciroli';
            simPar.nAgents          = 20;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 0 0.01];
        case 'polynomial'
            simPar.type             = 'polynomial';
            simPar.polyAgents       = 20;
            simPar.nAgents          = simPar.polyAgents;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 0;
            sampleGenome            = [0.1 0 0 0 0 0 0 0 -0.12 0 0 0 0 0 0.12];
        case 'sinusoid'
            simPar.type             = 'sinusoid';
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 0;
            simPar.sinusoidAgents   = 20;
            simPar.nAgents          = simPar.sinusoidAgents;
            sampleGenome            = [0.1 0 -0.0267 -5.4729 0.1886 0.7875 0.9981 34.0776 0.6417 1.3918 37.7451 0.9187 1.4766 15.6458 0.9927];
        case 'simpleNN'
            simPar.type             = 'simpleNN';
            simPar.nnSize           = simulations{si}.nnSize;
            simPar.polyAgents       = 0;
            simPar.nnAgents         = 20;
            simPar.sinusoidAgents   = 0;
            simPar.nAgents          = simPar.nnAgents;
            sampleGenome            = [0.1 0 rand(1,simulations{si}.genomeNetLength)];
            simPar.net              = struct();
            simPar.net.i_IB         = IB;
            simPar.net.i_OB         = OB;
            simPar.net.i_IW         = IW;
            simPar.net.i_LW         = LW;
    end
    %% Check lattice ratio (also checked inside sim_calc_cost but throws warnings)
    if ~lat_warning && simPar.camera_range/(simPar.seperation_range + simPar.collision_range)>=2
        simPar.camera_range = 1.95 * (simPar.seperation_range + simPar.collision_range);
        fprintf('WARNING: lattice ratio >= 2, limiting camera range to %.2fm\n', simPar.camera_range);
        lat_warning = true;
    end
    %% Check v_max vs a_max (6m/s^2)
    % Suppose 2 drones moving at each other at v_max
    % total v_diff          = 2*v_max
    % total deceleration    = 2 * a_max
    brake_distance = (2 * simPar.v_max)^2 / (2 * 2 * 6);    % v^2 / (2 * a)
    if ~v_max_warning && brake_distance > (simPar.camera_range - simPar.collision_range)
        fprintf('WARNING: maximum braking distance exceeds camera range (%.2fm vs. %.2fm)\n', brake_distance, simPar.camera_range);
        v_max_warning = true;
    end
    %% Check v_diff_max vs a_max
    % Suppose a drone moving forward at v_max, and then sets v_max backwards
    % v_diff = a_max / indi_speed_gain (2 * v_max = v_diff)
    saturation_v_max = 0.5 * 6 / 1.8;
    if ~acc_sat_warning && saturation_v_max/simPar.v_max < 1
        fprintf('WARNING: acceleration saturation point at %.2fm/s (%.0f%% of v_max)\n', saturation_v_max, saturation_v_max / simPar.v_max * 100);
        acc_sat_warning = true;
    end
    %% Do a sample simulation to get an estimate of the time
    pT              = tic;
    [cost, sArena, costStruct]  = sim_calc_cost(simPar, sampleGenome, false);
    %sArena.agents{1}.plotGlobalAttraction(-sArena.size(1):0.1:sArena.size(1),-sArena.size(2):0.1:sArena.size(2));
    simt            = toc(pT);
    fprintf(strcat(['\n\nSimulation ' simPar.type ' took ' num2str(simt) 's - performance: (cost: ' num2str(cost) ')\n\n']));
    fprintf('collision:\t%8.3f\nseparation:\t%8.3f\ndistance:\t%8.3f\nvelocity:\t%8.3f\n\n',costStruct.collisionCost, costStruct.seperationCost, costStruct.distanceCost, costStruct.velocityCost);
    %% Run GA
    agentType = simPar.type;
    psOptions = optimoptions('patternsearch',...
        'Display','iter',...
        'PlotFcn',@(optimValues,flag) plotResults(simPar, optimValues, [], flag, 'patternsearch'),...
        'StepTolerance',1e-9,...
        'ConstraintTolerance',1e-2,...
        ...%'Algorithm','sqp',...
        ...%'HonorBounds',false,...
        'UseParallel',1 ...
        );
    options = optimoptions('ga',...
        'FunctionTolerance', 1e-9, ...
        'ConstraintTolerance', 1e-2,...
        'MaxStallGenerations', 50,...
        'MaxGenerations', 200,...
        'PopulationSize',simulations{si}.popSize, ...
        'FitnessScalingFcn', {@fitscalingprop}, ...
        'Display','iter', ...
        'PlotFcn',@(options,state,flag) plotResults(simPar, options, state, flag, 'ga'),...
        'UseParallel',1,...
        ...%'OutputFcn',@gaOutputFun,...
        'CrossoverFraction',0.7,...
        'HybridFcn',{@patternsearch psOptions}...
        );
    alt_gads        = true;
    noMouseActions  = true;
    [x,fval,exitflag,output,population,scores] = ga(@(x) sim_calc_cost(simPar, x, false), length(simulations{si}.LB),[],[],[],[],simulations{si}.LB,simulations{si}.UB,[],options);
    %% save genome to file
    save(strcat(['./data/ga-' simulations{si}.type '-' simulations{si}.field(1).type '-' num2str(simPar.simTime) 's-' num2str(scores(1)) '.mat']),'x','fval','exitflag','output','population','scores','simPar');
    %% Genetic optimization finished, sim the winner and make video
    x_store{si}         = x;
    simPar_store{si}    = simPar;
end
%% Create videos
for i = 1:length(simulations)
    agentType                   = simPar_store{i}.type;
    [cost,uArena,costStruct]    = sim_calc_cost(simPar_store{i}, x_store{i}, true);
end
