close all hidden;
simTime         = 60;
nrOfEpochs      = 75;
generationSize  = 32;
trialSize       = 2;
familySize      = 20;
v_max           = 2;
collision_cost  = 1000;
velocity_cost   = 10;
cap_off         = 8;

% Create parpool of desired size
poolobj = gcp('nocreate');
if isempty(poolobj)
    parpool(4);  
end
generationCost = zeros(generationSize,1);
evolution = zeros(nrOfEpochs,generationSize);
clear bestGenomes;
bestGenomes = zeros(nrOfEpochs,cap_off,13);
initGenomes = zeros(generationSize,13);
%bestGenomes{1}  =  [0 0 0 0 0 0 -0.12 0 0 0 0 0 0.12];
for i=1:generationSize
    initGenomes(i,:) = 0.05 * randn(1,13);
end
for g=1:nrOfEpochs
    genT = tic;
    if g==1
        nextGenomes = mutateGenome(generationSize, initGenomes, 0.001, 0.0001);
    else
        nextGenomes = mutateGenome(generationSize, squeeze(bestGenomes(g,:,:)), 0.05, 0.001);
    end
    clear uArena simT movT add_args;
    uArena{generationSize}   = {};
    names = {generationSize};
    for i=1:generationSize
        names{i} = strcat('generation-',num2str(g),'-specimen-',num2str(i));
    end
    simT{generationSize}     = {};
    movT{generationSize}     = {};
    add_args{generationSize} = {};
    t                       = zeros(generationSize,1);
    tm                      = zeros(generationSize,1);
    %for i=1:length(names)
    parfor i=1:length(names)
        % Simulation options
        uArena{i}               = Mission(num2str(i),'cyberzooCW',add_args{i});
        uArena{i}.T             = simTime;
        uArena{i}.dt            = 1/15;
        uArena{i}.nAgents       = 0;
        uArena{i}.nnAgents      = 0;
        uArena{i}.circle_radius = 5;
        uArena{i}.separation_range = 1.5;
        uArena{i}.nAgents       = familySize;
        uArena{i}.polyAgents    = familySize;
        uArena{i}.init          = 'rect';
        uArena{i}.size          = [7 7];
        uArena{i}.agent_conf    = struct('v_max',v_max, 'genome', squeeze(nextGenomes(i,:)));
        % Save/Display options
        uArena{i}.print         = 0;   % Print ETA and % if larger than 1 it shown every (rounded to factor of nT) i-th percentage without erasing previous line
        uArena{i}.save          = 1;   % Save data to .mat file
        generationCost(i)       = 0;
        %fprintf(strcat(['Simulation ' num2str(i) ': Initialised arena and agents\n']));
        for s=1:trialSize
            simT{i} = tic; uArena{i}.Simulate(); t(i) = toc(simT{i});
            for j=1:familySize
                generationCost(i) = generationCost(i) + velocity_cost * uArena{i}.agents{j}.cost;
            end
        end
        generationCost(i) = generationCost(i) + collision_cost * sum(sum(uArena{i}.collisions));
        generationCost(i) = generationCost(i) / (simTime * familySize * trialSize);
        %fprintf(strcat(['Sim (cost: ', num2str(generationCost(i)) ,') ' num2str(i) ' took ' uArena{i}.sec2time(round(t(i))) 's at ' num2str(round(uArena{i}.T/t(i),2)) 'x speed\n']));
    end
    [Y,I] = sort(generationCost);
    evolution(g,:) = Y;
    figure(1);
    subplot(1,2,1);
    hold all;
    plot(g*ones(1,generationSize),evolution(g,:),'o');
    hold off;
    axis([0 g 0 max(min(evolution(1:g,:),[],2))]);
    drawnow;
    xlabel('Generation');
    ylabel('Performance');
    n = 1;
    subplot(1,2,2);
    hold all;
    for f=1:cap_off
        bestGenomes(g+1,f,:) = uArena{I(f)}.agents{1}.genome;
        plot(1:13,squeeze(bestGenomes(g+1,f,:)),'o');
    end
    hold off;
    xlabel('genome order');
    ylabel('coefficient');
    drawnow;
    fprintf(strcat(['Generation ' num2str(g) '(best: ', num2str(min(Y)) ,')\ttook ' uArena{1}.sec2time(round(toc(genT))) 's\n']));
end
uArena{I(1)}.agents{1}.plotPolynomial(2);
uArena{I(2)}.agents{1}.plotPolynomial(3);
uArena{I(3)}.agents{1}.plotPolynomial(4);
uArena{I(4)}.agents{1}.plotPolynomial(5);
clear i simT;
delete(gcp('nocreate'));
clear poolobj;
for j=1:length(names)
    if uArena{j}.save == 1
        tmp = uArena{j};
        save(strcat('./data/',uArena{j}.name,'.mat'),'tmp');
    end
end