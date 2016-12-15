function [varargout] = sim_calc_cost( simPar, genome, makeVideo)
%UNTITLED Simulate according to simPar and calculate cost
%   For use with ga toolbox
rng('default');
distanceCost    = 0;
collisionCost   = 0;
seperationCost  = 0;
velocityCost    = 0;
simPar.camera_range     = (simPar.seperation_range) * genome(1);
genome(1)               = [];
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
    uArena.boc                  = simPar.boc;
    uArena.moving_axes          = simPar.moving_axes;
    % What type of agents
    uArena.nAgents              = simPar.nAgents;
    uArena.nnAgents             = simPar.nnAgents;
    uArena.polyAgents           = simPar.polyAgents;
    uArena.sinusoidAgents       = simPar.sinusoidAgents;
    uArena.field                = simPar.field;
    % Simulation dependant options
    if simPar.camera_range/(simPar.seperation_range)>2
        simPar.camera_range = 1.95 * (simPar.seperation_range);
        fprintf('WARNING: lattice ratio > 2, limiting camera range to %9.2f\n', simPar.camera_range);
    end
    switch(simPar.type)
        case 'simpleNN'
            fakeNet             = struct();
            fakeNet.numLayers   = length(simPar.nnSize);
            fakeNet.IW          = genome(simPar.net.i_IW+2)';
            fakeNet.IB          = genome(simPar.net.i_IB+2)';
            fakeNet.LW          = genome(simPar.net.i_LW+2);
            fakeNet.OB          = genome(simPar.net.i_OB+2);
            uArena.agent_conf   = struct('seperation_range', simPar.seperation_range,'collision_range', simPar.collision_range,'cam_range', simPar.camera_range,'v_max',simPar.v_max, 'genome', genome, 'net', fakeNet);
        otherwise
            uArena.agent_conf   = struct('seperation_range', simPar.seperation_range,'collision_range', simPar.collision_range,'cam_range', simPar.camera_range,'v_max',simPar.v_max, 'genome', genome);
    end
    % Save/Display options
    uArena.print                = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena.save                 = 1;   % Save data to .mat file
    % Simulate
    uArena.Simulate();
    % Calculate cost
    dcX             = 0:1/(simPar.simTime * simPar.fps):1;
    dcX(1)          = [];
    dcW             = sqrt(1-(dcX-1).^4);
    dcW             = dcW' / sum(dcW);
    distanceCost    = distanceCost  + sum(dcW .* uArena.distance_cost);
    collisionCost   = collisionCost + (1 - uArena.t/(2 * simPar.simTime * simPar.fps)) * sum(sum(uArena.collisions));
    seperationCost  = seperationCost + sum(dcW .* uArena.seperation_cost.^2);
    velocityCost    = velocityCost + sum(dcW .* uArena.velocity_cost);
    % Create video (optional)
    if makeVideo
        createVideo(uArena);
    end
end
% Normalize cost wrt simulation parameters
velocityCost    = simPar.velocity_cost      * velocityCost      / (simPar.nAgents * simPar.trialSize);
distanceCost    = simPar.distance_cost      * distanceCost      / (simPar.nAgents * simPar.trialSize);
collisionCost   = simPar.collision_cost     * collisionCost     / (simPar.simTime * simPar.fps * simPar.nAgents * simPar.trialSize);
seperationCost  = simPar.seperation_cost    * seperationCost    / (simPar.nAgents * simPar.trialSize);
varargout{1}    = seperationCost + collisionCost + distanceCost + velocityCost;
if nargout > 1
    varargout{2}                    = uArena;
    if nargout > 2
        costStruct                  = struct();
        costStruct.velocityCost     = velocityCost;
        costStruct.distanceCost     = distanceCost;
        costStruct.collisionCost    = collisionCost;
        costStruct.seperationCost   = seperationCost;
        varargout{3}                = costStruct;
    end
end
end
