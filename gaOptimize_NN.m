pT      = tic;
[cost]  = sim_calc_cost([0 0 0 0 0 0 -0.12 0 0 0 0 0 0.12]);
simt    = toc(pT);
options = optimoptions('ga','PopulationSize',100, 'Display','iter', 'PlotFcn',{@gaplotbestf_scale @gaplotbestindiv @gaplotgenealogy},'UseParallel',1);
LB      = -1 * ones(1,16);
UB      =  1 * ones(1,16);
fprintf(strcat(['Simulation took ' num2str(simt) 's - pinciroliPerformance: (cost: ' num2str(cost) ')\n\n']));
[x,fval,exitflag,output,population,scores] = ga(@sim_calc_cost_nn, 16,[],[],[],[],LB,UB,[],options);

%% Genetic optimization finished, sim the winner and make video
global simPar;
simPar = struct(...
    'simTime',              20, ...
    'trialSize',            3, ...
    'fps',                  15, ...
    'nAgents',              4, ...
    'nnAgents',             4, ...
    'circle_radius',        4, ...
    'seperation_range',     1.5, ...
    'init',                 'rect', ...
    'size',                 [4 4], ...
    'v_max',                2, ...
    'distance_cost',        0.2, ...
    'velocity_cost',        1, ...
    'collision_cost',       1 ...
    );
simPar.mission = {'cyberzooCW' 'cyberzooCCW' 'cyberzooBucket'};
for i=1:length(simPar.mission)
    uArena                      = Mission(num2str(1),simPar.mission{i},{});
    uArena.T                    = simPar.simTime;
    uArena.dt                   = simPar.fps^(-1);
    uArena.nAgents              = simPar.nAgents;
    uArena.nnAgents             = simPar.nnAgents;
    uArena.circle_radius        = simPar.circle_radius;
    uArena.separation_range     = simPar.seperation_range;
    uArena.init                 = simPar.init;
    uArena.size                 = simPar.size;
    uArena.agent_conf           = struct('v_max',simPar.v_max, 'genome', x);
    % Save/Display options
    uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena.save                 = 1;   % Save data to .mat file
    distanceCost = 0;
    velocityCost = 0;
    collisionCost = 0;
    %fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n']));
    simT = tic; uArena.Simulate(); t = toc(simT);
    for j=1:simPar.nAgents
        velocityCost = velocityCost + simPar.velocity_cost * uArena.agents{j}.vel_cost;
        distanceCost = distanceCost + simPar.distance_cost * uArena.agents{j}.dist_cost;
    end
    collisionCost = collisionCost + simPar.collision_cost * sum(sum(uArena.collisions));
    velocityCost = velocityCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
    distanceCost = distanceCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
    createVideo(uArena);
end