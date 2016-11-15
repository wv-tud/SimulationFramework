function [ totCost ] = sim_calc_cost( simPar, genome, makeVideo )
%UNTITLED Simulate according to simPar and calculate cost
%   For use with ga toolbox
rng('default');
distanceCost    = 0;
velocityCost    = 0;
collisionCost   = 0;
for s=1:simPar.trialSize
    % Simulation options
    if ~isfield(simPar,'mission')
       uArena              	= Mission(num2str(1),'cyberzooCW',{}); 
    else
        if ~isa(simPar.mission,'cell')
           uArena           = Mission(num2str(1),simPar.mission,{});  
        else
           missionIndex     = s - floor((s-1) / size(simPar.mission,2)) * size(simPar.mission,2);
           uArena           = Mission(num2str(1),simPar.mission{missionIndex},{});
        end
    end 
    uArena.typeName             = simPar.type;
    uArena.T                    = simPar.simTime;
    uArena.dt                   = simPar.fps^(-1);
    uArena.separation_range     = simPar.seperation_range;
    uArena.init                 = simPar.init;
    uArena.size                 = simPar.size;
    % What type of agents
    uArena.nAgents              = simPar.nAgents;
    uArena.nnAgents             = simPar.nnAgents;
    uArena.polyAgents           = simPar.polyAgents;
    uArena.sinusoidAgents       = simPar.sinusoidAgents;
    % Simulation dependant options
    switch(simPar.type)
        case 'simpleNN'
            simPar.net                  = setwb(simPar.net,genome(2:end));
            uArena.agent_conf           = struct('v_max',simPar.v_max, 'genome', genome, 'net', @(x) simPar.net(x));
        otherwise
            uArena.agent_conf           = struct('v_max',simPar.v_max, 'genome', genome);
    end
    % Save/Display options
    uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena.save                 = 1;   % Save data to .mat file
    % Simulate
    uArena.Simulate();
    % Calculate cost
    for j=1:simPar.nAgents
        velocityCost = velocityCost + uArena.agents{j}.vel_cost;
        %distanceCost = distanceCost + uArena.agents{j}.dist_cost;
    end
    uArena.agents{1}.plotGlobalAttraction(-7:0.1:7,-7:0.1:7);
    distanceCost    = distanceCost  + uArena.distance_cost;
    collisionCost   = collisionCost + sum(sum(uArena.collisions));
    % Create video (optional)
    if makeVideo
        createVideo(uArena);
    end
end
% Normalize cost wrt simulation parameters
velocityCost    = simPar.velocity_cost * velocityCost / (simPar.simTime * simPar.fps * simPar.nAgents * simPar.trialSize);
distanceCost    = simPar.distance_cost * distanceCost / (simPar.simTime * simPar.fps * simPar.nAgents * simPar.trialSize);
collisionCost   = simPar.collision_cost * collisionCost / (simPar.simTime * simPar.fps * simPar.nAgents * simPar.trialSize);
%velocityCost
%distanceCost
%collisionCost
totCost         = collisionCost + velocityCost + distanceCost;
end
