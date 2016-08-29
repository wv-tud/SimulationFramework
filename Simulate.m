close all hidden; clear uArena;
names                   = {'cyberzoo'};
add_args                = {0};
v_max                   = 4;
uArena{length(names)}   = {};
simT{length(names)}     = {};
movT{length(names)}     = {};
t                       = zeros(length(names),1);
tm                      = zeros(length(names),1);
for i=1:length(names)  
    % Simulation options
    uArena{i}               = Mission(num2str(i),names{i},add_args{i});
    uArena{i}.T             = 200;
    uArena{i}.dt            = 1/15;
    uArena{i}.nAgents       = 4;
    uArena{i}.init          = 'square';
    uArena{i}.size          = [8 8];
    uArena{i}.agent_conf    = struct('v_max',v_max);
    % Save/Display options
    uArena{i}.print         = 5;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena{i}.save          = 0;   % Save data to .mat file

    fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n'])); 
    simT{i} = tic; uArena{i}.Simulate(); t(i) = toc(simT{i});
    fprintf(strcat(['Simulation ' num2str(i) ' took ' uArena{i}.sec2time(round(t(i))) 's at ' num2str(round(uArena{i}.T/t(i),2)) 'x speed\n']));
end
clear i simT;
for j=1:length(names)
    if uArena{j}.save == 1
        tmp = uArena{j}; %#ok
        save(strcat('./data/',uArena{j}.name,'.mat'),'tmp');
    end
    % Create movie/frames
    visObj              = visualArena(uArena{j});
    % Figure options
    visObj.showFig      = 1;  % Show = 3x faster
    visObj.p_fov        = 0;
    visObj.p_circ       = 0;
    visObj.p_head       = 2; 
    visObj.p_label      = 0;
    visObj.p_mov_axe    = 0;
    visObj.p_axe_lim    = [-4 4 -4 4];  % Cyberzoo
    fprintf(strcat(['VisualArena ' num2str(j) ': Initialised videoWriter\n'])); 
    movT{j} = tic; visObj.build(); tm(j) = toc(movT{j});
    fprintf(strcat(['VisualArena ' num2str(j) ' took ' uArena{j}.sec2time(round(tm(j))) 's at ' num2str(round(uArena{j}.T/tm(j),2)) 'x speed\n']));
end
clear j movT visObj tmp;