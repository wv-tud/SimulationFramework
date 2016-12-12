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
    visObj.p_label      = 1;
    visObj.resolution   = [720 720];
    if ~uArena.moving_axes
        visObj.p_axe_lim    = [-uArena.size(1) uArena.size(1) -uArena.size(2) uArena.size(2)];
    end
    fprintf(strcat(['VisualArena: Initialised videoWriter\n'])); 
    movT = tic; visObj.build(); tm = toc(movT);
    fprintf(strcat(['VisualArena took ' uArena.sec2time(round(tm)) 's at ' num2str(round(uArena.T/tm,2)) 'x speed\n']));
end

