function [ totCost ] = sim_calc_cost( simPar, genome, makeVideo )
%UNTITLED Simulate according to simPar and calculate cost
%   For use with ga toolbox
rng('default');
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
    uArena.type                 = simPar.type;
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
        simPar.net              = setwb(simPar.net,genome(2:end));
        uArena.agent_conf       = struct('v_max',simPar.v_max, 'genome', genome, 'net', @(x) simPar.net(x));
    else
        uArena.agent_conf       = struct('v_max',simPar.v_max, 'genome', genome);
    end
    
    % Save/Display options
    uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena.save                 = 1;   % Save data to .mat file
    %fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n']));
    
    uArena.Simulate();
    for j=1:simPar.nAgents
        velocityCost = velocityCost + uArena.agents{j}.vel_cost;
        distanceCost = distanceCost + uArena.agents{j}.dist_cost;
    end
    if makeVideo
        createVideo(uArena);
    end
    %if sum(sum(uArena.collisions)) > 0
    %    fprintf(strcat([uArena.mission_type ' collisions: ' num2str(sum(sum(uArena.collisions))) '\n']));
    %end
    collisionCost = collisionCost + sum(sum(uArena.collisions));
end
velocityCost  = simPar.velocity_cost * velocityCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
distanceCost  = simPar.distance_cost * distanceCost / (simPar.simTime * simPar.nAgents * simPar.trialSize);
collisionCost = simPar.collision_cost * collisionCost;
%velocityCost
%distanceCost
%collisionCost
totCost = collisionCost + velocityCost + distanceCost;
end
