MAX_NEIGHBOURS = 5;

sampleSize = 0;
for u=1:length(uArena)
    sampleSize = sampleSize + length(uArena{u}.agents) * (length(uArena{u}.agents{1}.pos(:,1))-1);
end
input   = zeros(sampleSize, MAX_NEIGHBOURS*2);
output  = zeros(sampleSize, 2);
t_avg   = 0;
i       = 0;
p       = 0;
iterT   = tic;
totalT  = tic;
print   = 1;
for u=1:length(uArena)
    for a=1:length(uArena{u}.agents)
        for t=1:(length(uArena{u}.agents{a}.pos(:,1))-1)
            i = i + 1;
            neighbours = zeros(size(uArena{u}.agents{a}.neighbours{t},1),4);
            for n=1:size(uArena{u}.agents{a}.neighbours{t},1)
                neighbours(n,3:4)   = uArena{u}.agents{a}.neighbours{t}(n,3:4) - uArena{u}.agents{a}.pos(t+1,1:2);
                neighbours(n,1)     = max(0,min(1,uArena{u}.agents{a}.collision_range / norm(neighbours(n,3:4))));
                neighbours(n,2)     = max(0,min(1,(atan2(neighbours(n,4),neighbours(n,3)) + pi())/(2*pi())));
            end
            neighbours = sort(neighbours,1,'descend');
            if size(neighbours,1)>MAX_NEIGHBOURS
                neighbours = neighbours(1:MAX_NEIGHBOURS,:);
            end
            % Radius + angle (normalized)
            input(i,1:(2*size(neighbours,1))) =  reshape(neighbours(:,1:2)',1,numel(neighbours(:,1:2)));
            % X + Y
            %input(end+1,1:(2*size(neighbours,1))) =  reshape(neighbours(:,3:4)',1,numel(neighbours(:,3:4)));
            output(i,:) = [norm(uArena{u}.agents{a}.u_d_decom.L(t,1:2))/uArena{1}.agents{1}.v_max (atan2(uArena{u}.agents{a}.u_d_decom.L(t,2),uArena{u}.agents{a}.u_d_decom.L(t,1)) + pi())/(2*pi())];
            % Show progress
            if print > 0 && print < 1
                t_el        = toc(iterT);                       % Get elapsed time
                t_avg       = (i-1)/(i)*t_avg + 1/i*t_el;       % Calculate new avg time
                iterT       = tic;                              % Set new time
                perc        = i/sampleSize;                     % Get iteration percentage
                ETA         = round(t_avg*(sampleSize-i));      % Calculate time remaining
                text        = strcat([char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' sec2time(ETA) ')\n']);
                if i>1
                    fprintf(char(repmat('\b',1,prev_l-2)));
                end
                fprintf (text);
                prev_l = length(text);
            elseif print>=1 && ~mod(i,print*round(sampleSize/100))
                p           = p + 1;
                t_el        = toc(iterT);                       % Get elapsed time
                t_avg       = (p-1)/(p)*t_avg + 1/p*t_el;       % Calculate new avg time
                iterT       = tic;                              % Set new time
                perc        = i/sampleSize;                     % Get iteration percentage
                ETA         = round(t_avg*(1-perc)*100);        % Calculate time remaining
                text        = strcat(['Dataset: ' char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' sec2time(ETA) ')\n']);
                fprintf (text);
            end
        end
    end
end
toc(totalT);