close all hidden;
clear uArena;
names           = {'cyberzooCW'};
add_args        = {0 0 0};
v_max           = 1;
% Simulate
simTime     = 30;
simAgents   = 6;
Simulate;
createVideos;
% uArena{1}.agents{1}.plotGlobalAttraction(-4:0.05:4,-4:0.05:4);
% uArena{2}.agents{1}.plotGlobalAttraction(-4:0.05:4,-4:0.05:4);
% uArena{3}.agents{1}.plotGlobalAttraction(-4:0.05:4,-4:0.05:4);