uArena{length(names)}   = {};
simT{length(names)}     = {};
movT{length(names)}     = {};
t                       = zeros(length(names),1);
tm                      = zeros(length(names),1);
if ~exist('netAgents','var')
    netAgents = 0;
end
if ~exist('simTime','var')
    simTime = 60;
end
if ~exist('simAgents','var')
    simAgents = 8;
end
for i=1:length(names)  
    % Simulation options
    uArena{i}               = Mission(num2str(i),names{i},add_args{i});
    uArena{i}.T             = simTime;
    uArena{i}.dt            = 1/15;
    uArena{i}.nAgents       = simAgents;
    uArena{i}.nnAgents      = netAgents;
    uArena{i}.init          = 'random';
    uArena{i}.size          = [8 8];
    uArena{i}.agent_conf    = struct('v_max',v_max);
    % Save/Display options
    uArena{i}.print         = 2;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena{i}.save          = 1;   % Save data to .mat file

    fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n'])); 
    simT{i} = tic; uArena{i}.Simulate(); t(i) = toc(simT{i});
    fprintf(strcat(['Simulation ' num2str(i) ' took ' uArena{i}.sec2time(round(t(i))) 's at ' num2str(round(uArena{i}.T/t(i),2)) 'x speed\n']));
end
clear i simT;