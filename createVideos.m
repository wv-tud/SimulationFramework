%uArena{1}.agents{1}.plotVelocityComponents();
%uArena{1}.agents{1}.plotGlobalAttraction(uArena{1}.p_axe_lim(1):0.3:uArena{1}.p_axe_lim(2),uArena{1}.p_axe_lim(3):0.3:uArena{1}.p_axe_lim(4),uArena{1}.c_fun(0));
for j=1:length(names)
    % Create movie/frames
    visObj              = visualArena(uArena{j});
    % Figure options
    visObj.showFig      = 1;  % Show = 3x faster
    visObj.p_fov        = 2;
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