simPar = struct(...
    'type',                 '',...
    'simTime',              30, ...
    'trialSize',            1, ...
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
sampleGenome            = [0.000012 0.12];

% Simulation options
uArena                      = Mission(num2str(1),'cyberzooCW',{});
uArena.T                    = simPar.simTime;
uArena.dt                   = simPar.fps^(-1);
uArena.nAgents              = simPar.nAgents;
uArena.nnAgents             = simPar.nnAgents;
uArena.circle_radius        = simPar.circle_radius;
uArena.separation_range     = simPar.seperation_range;
uArena.polyAgents           = simPar.polyAgents;
uArena.init                 = simPar.init;
uArena.size                 = simPar.size;
if strcmp(simPar.type,'simpleNN')
    uArena.net              = setwb(simPar.net,sampleGenome(2:end));
end
uArena.agent_conf           = struct('v_max',simPar.v_max, 'genome', sampleGenome);
% Save/Display options
uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
uArena.save                 = 1;   % Save data to .mat file
%fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n']));
simT = tic; uArena.Simulate(); t = toc(simT);