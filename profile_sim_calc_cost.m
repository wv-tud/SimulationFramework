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
    'size',                 [4 4], ...
    'v_max',                2, ...
    'distance_cost',        0.2, ...
    'velocity_cost',        1, ...
    'collision_cost',       1 ...
    );
simPar.mission = {'cyberzooBucket'};
simPar.type             = 'pinciroli';
simPar.nAgents          = 9;
simPar.polyAgents       = 0;
simPar.nnAgents         = 0;
simPar.sinusoidAgents   = 0;
sampleGenome            = [0.12 0.12];
[cost]                  = sim_calc_cost(simPar, sampleGenome);