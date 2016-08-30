close all hidden;
clear uArena;
names       = {'cyberzooCW' 'cyberzooCCW' 'cyberzooBucket' 'cyberzooBucket'};
add_args    = {0 0 0 0};
v_max = 3;
% Create parpool of desired size
poolobj = gcp('nocreate');
if isempty(poolobj)
    poolobj = parpool(min(length(names),8));  
else
    if poolobj.NumWorkers > length(names)
        delete(gcp('nocreate'));
        poolobj = parpool(min(length(names),8));
    end
end
uArena{length(names)}   = {};
simT{length(names)}     = {};
movT{length(names)}     = {};
t                       = zeros(length(names),1);
tm                      = zeros(length(names),1);
parfor i=1:length(names)  
    % Simulation options
    uArena{i}               = Mission(num2str(i),names{i},add_args{i});
    uArena{i}.T             = 3600;
    uArena{i}.dt            = 1/15;
    uArena{i}.nAgents       = 8;
    uArena{i}.init          = 'random';
    uArena{i}.size          = [4 4];
    uArena{i}.agent_conf    = struct('v_max',v_max);
    % Save/Display options
    uArena{i}.print         = 2;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
    uArena{i}.save          = 1;   % Save data to .mat file

    fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n'])); 
    simT{i} = tic; uArena{i}.Simulate(); t(i) = toc(simT{i});
    fprintf(strcat(['Simulation ' num2str(i) ' took ' uArena{i}.sec2time(round(t(i))) 's at ' num2str(round(uArena{i}.T/t(i),2)) 'x speed\n']));
end
clear i simT;
delete(gcp('nocreate'));
% uArena{1}.agents{1}.plotVelocityComponents();
% uArena{1}.agents{1}.plotGlobalAttraction(uArena{1}.p_axe_lim(1):0.3:uArena{1}.p_axe_lim(2),uArena{1}.p_axe_lim(3):0.3:uArena{1}.p_axe_lim(4),uArena{1}.c_fun(0));
% for j=1:length(names)
%     if uArena{j}.save == 1
%         tmp = uArena{j}; %#ok
%         save(strcat('./data/',uArena{j}.name,'.mat'),'tmp');
%     end
%     % Create movie/frames
%     visObj              = visualArena(uArena{j});
%     % Figure options
%     visObj.showFig      = 1;  % Show = 3x faster
%     visObj.p_fov        = 0;
%     visObj.p_circ       = 0;
%     visObj.p_head       = 2; 
%     visObj.p_label      = 0;
%     visObj.p_mov_axe    = 0;
%     visObj.p_axe_lim    = [-52.5 52.5 -34 34];  % Soccer field
%     fprintf(strcat(['VisualArena ' num2str(j) ': Initialised videoWriter\n'])); 
%     movT{j} = tic; visObj.build(); tm(j) = toc(movT{j});
%     fprintf(strcat(['VisualArena ' num2str(j) ' took ' uArena{j}.sec2time(round(tm(j))) 's at ' num2str(round(uArena{j}.T/tm(j),2)) 'x speed\n']));
% end
% clear j movT visObj tmp;