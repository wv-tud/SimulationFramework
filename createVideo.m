function [  ] = createVideo( uArena )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Create movie/frames
    visObj              = visualArena(uArena);
    % Figure options
    visObj.showFig      = 1;  % Show = 3x faster
    visObj.p_fov        = 2;
    visObj.p_circ       = 0;
    visObj.p_head       = 2; 
    visObj.p_label      = 0;
    visObj.resolution   = [720 720];
    visObj.p_mov_axe    = 0;
    visObj.p_axe_lim    = [-4 4 -4 4];  % Cyberzoo
    fprintf(strcat(['VisualArena: Initialised videoWriter\n'])); 
    movT = tic; visObj.build(); tm = toc(movT);
    fprintf(strcat(['VisualArena took ' uArena.sec2time(round(tm)) 's at ' num2str(round(uArena.T/tm,2)) 'x speed\n']));
end

