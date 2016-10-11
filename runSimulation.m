close all hidden; clear uArena;
names       = {'cyberzooBucket'};
add_args    = {0};
v_max       = 4;
writeVideo  = 1;
simTime     = 60;
simAgents   = 8;
Simulate;

if writeVideo == 1
    createVideos;
end