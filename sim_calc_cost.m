function [ totCost ] = sim_calc_cost( genome )
%UNTITLED Simulate according to simPar and calculate cost
%   For use with ga toolbox
simPar = struct(...
    'simTime',              20, ... 
    'trialSize',            3, ... 
    'fps',                  15, ...
    'nAgents',              6, ...
    'polyAgents',           6, ...
    'nnAgents',             0, ...
    'circle_radius',        2, ...
    'seperation_range',     1.5, ...
    'init',                 'rect', ...
    'size',                 [3 3], ...
    'v_max',                2, ...
    'distance_cost',        0.2, ...
    'velocity_cost',        1, ...
    'collision_cost',       1 ...
);
simPar.mission = {'cyberzooCW' 'cyberzooCCW' 'cyberzooBucket'};
distanceCost = 0;
velocityCost = 0;
collisionCost = 0;
for s=1:simPar.trialSize
    % Simulation options
    if ~isfield(simPar,'mission')
       uArena                      = Mission(num2str(1),'cyberzooCW',{}); 
    else
        if ~isa(simPar.mission,'cell')
           uArena                      = Mission(num2str(1),simPar.mission,{});  
        else
           missionIndex = s - floor((s-1) / size(simPar.mission,2)) * size(simPar.mission,2);
           uArena                      = Mission(num2str(1),simPar.mission{missionIndex},{});
        end
    end  
    uArena.T                    = simPar.simTime;
    uArena.dt                   = simPar.fps^(-1);
    uArena.nAgents              = simPar.nAgents;
    uArena.nnAgents             = simPar.nnAgents;
    uArena.circle_radius        = simPar.circle_radius;
    uArena.separation_range     = simPar.seperation_range;
    uArena.polyAgents           = simPar.polyAgents;
    uArena.init                 = simPar.init;
    uArena.size                 = simPar.size;
    uArena.agent_conf           = struct('v_max',simPar.v_max, 'genome', genome);
    % Save/Display options
    uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena.save                 = 1;   % Save data to .mat file
    %fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n']));
    
    simT = tic; uArena.Simulate(); t = toc(simT);
    for j=1:simPar.nAgents
        velocityCost = velocityCost + uArena.agents{j}.vel_cost;
        distanceCost = distanceCost + uArena.agents{j}.dist_cost;
    end
    %if sum(sum(uArena.collisions)) > 0
    %    fprintf(strcat([uArena.mission_type ' collisions: ' num2str(sum(sum(uArena.collisions))) '\n']));
    %end
    collisionCost = collisionCost + sum(sum(uArena.collisions));
end
velocityCost = velocityCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
distanceCost = distanceCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
totCost = simPar.collision_cost * collisionCost + simPar.velocity_cost * velocityCost + simPar.distance_cost * distanceCost;
end
