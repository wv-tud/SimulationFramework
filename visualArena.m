classdef visualArena < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        frame_save  = 0;                    % Save seperate frames ISO create movie
        showFig     = 1;                    % Show figure during build
        p_fov       = 0;                    % Print field of view line of every agent
        p_head      = 2;                    % Print heading line: 0=off;1=range;2=circle
        p_circ      = 0;                    % Print circle of 0.5m around every agent
        p_label     = 0;                    % Print agent id
        p_mov_axe   = 0;                    % Print using moving axes
        p_axe_lim   = [-52.5 52.5 -34 34];  % Limits zooming on the axe [xlim ylim]
        diskType    = 'hdd';                % Whether to write to HDD or SSD
        resolution  = [1000 1000];          % Figure resolution (width will be adapted to aspect ratio of p_axe_lim)
        print       = 1;                    % Print progress to command window
        % Non optional properties
        moviefile;                          % Name of movie file
        arenaVars;                          % Structure containing relevant arena variables
        % Handles
        fig_table   = {};                   % Figure table handle
        movie;                              % VideoWriter object
        fig;                                % Figure handle
        axes;                               % Axes handle
        fh_drone;
        fh_circles;
        fh_head;
        fh_fovL;
        fh_fovR;
        fh_label;
        fh_collisions = {};
        fh_cpos;
    end
    
    methods
        function obj = visualArena(arena)
            obj.arenaVars.cam_range         = arena.agents{1}.cam_range;          % Save parameters to local
            obj.arenaVars.cam_dir           = arena.agents{1}.cam_dir;
            obj.arenaVars.cam_fov           = arena.agents{1}.cam_fov;
            obj.arenaVars.collision_range   = arena.agents{1}.collision_range;
            obj.arenaVars.nAgents           = arena.nAgents;
            obj.arenaVars.agents            = arena.agents;
            obj.arenaVars.c_pos             = arena.c_pos;
            obj.arenaVars.collisions        = arena.collisions;
            obj.arenaVars.a_headings        = arena.a_headings;
            obj.arenaVars.a_positions       = arena.a_positions;
            obj.arenaVars.dt                = arena.dt;
            obj.arenaVars.T                 = arena.T;
            if obj.frame_save~=1
                name        = strcat([arena.name '.1']);
                file_clear  = 0;
                version     = 1;
                while file_clear == 0 % Rename to -v1.1,v1.2,...,v1.N when file v1 exists
                    if exist(strcat(['./movies_' obj.diskType '/' name '.avi']), 'file') == 2
                        version = version + 1;
                        i       = 1;
                        foundV  = 0;
                        while foundV~=1
                            if name(end-i) == '.'
                                foundV = 1;
                            else
                                i = i+1;
                            end
                        end
                        name = strcat([name(1:end-i) num2str(version)]);
                    else
                        file_clear = 1;
                    end
                end
                obj.moviefile       = strcat('./movies_',obj.diskType,'/',name,'.avi');
                obj.movie           = VideoWriter(obj.moviefile,'Motion JPEG AVI');
                obj.movie.Quality   = 75;
                obj.movie.FrameRate = 1/arena.dt;
                open(obj.movie);
            end
            % Create figure
            if ~isempty(obj.p_axe_lim)
                if obj.resolution(1)/obj.resolution(2) ~= round((obj.p_axe_lim(2)-obj.p_axe_lim(1))/(obj.p_axe_lim(4)-obj.p_axe_lim(3)),3) % Do a resolution/axe_limit aspect ratio test
                    obj.resolution(1) = obj.resolution(1).*round((obj.p_axe_lim(2)-obj.p_axe_lim(1))/(obj.p_axe_lim(4)-obj.p_axe_lim(3)),3);
                end
            end
            obj.resolution(1) = obj.resolution(1) + 200; % Increase figure width to include table
            obj.fig             = figure('Position',[0 0 obj.resolution(1) obj.resolution(2)]);
            obj.fig_table{1}    = figureTable(0,700,obj.resolution,{'\eta_{v}','\eta_{\Psi}','\eta_{camera}','r_{camera}','FOV','\Theta_{max}','v_{max}','r_{seperation}','\Deltat','T_{sim}','N_{agents}'},{strcat([num2str(100*arena.agents{1}.v_acc) '%']),strcat([num2str(100*arena.agents{1}.yaw_acc) '%']),strcat([num2str(100*arena.agents{1}.cam_acc) '%']),arena.agents{1}.cam_range,rad2deg(arena.agents{1}.cam_fov),rad2deg(arena.agents{1}.th_max),arena.agents{1}.v_max,arena.agents{1}.seperation_range,arena.dt,arena.T,arena.nAgents});
            obj.fig_table{2}    = figureTable(0,550,obj.resolution,{'collisions' 't'},{'0' '0.0'});
            obj.axes            = axes('position',[0.03 0.05 (0.94*(obj.resolution(1)-200))/obj.resolution(1) 0.90]);
            set(obj.fig,'PaperPositionMode','manual');
            set(obj.fig,'PaperPosition',[100 100 obj.resolution(1) obj.resolution(2)]);
            set(obj.fig,'Renderer','opengl','DockControls','off','MenuBar','none','ToolBar','none','Resize','off','InvertHardcopy','off');
            xlabel('Position [m]');
            ylabel('Position [m]');
            title(arena.name);
            % Preallocate arrays
            obj.fh_collisions{obj.arenaVars.nAgents} = 0;
        end
        
        function filename = build(obj)
            if obj.showFig==0
                set(obj.fig,'Visible','off'); 
            end
            nT = obj.arenaVars.T/obj.arenaVars.dt;
            if obj.print>0 
                iterT = tic;  % Start timer for first iteration
                t_ar = zeros(nT,1);
            end
            for ti=1:nT
                obj.printAgents(ti,squeeze(obj.arenaVars.a_positions(ti,:,:)),squeeze(obj.arenaVars.a_headings(ti,:,:)),sum(sum(obj.arenaVars.collisions(1:ti,:))));
                if obj.print==1
                    t_ar(ti)    = toc(iterT);                       % Get elapsed time
                    iterT       = tic;                              % Set new time
                    perc        = ti/nT;                            % Get iteration percentage 
                    ETA         = round(mean(t_ar(1:ti))*(nT-ti));  % Calculate time remaining
                    text        = strcat([char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' obj.sec2time(ETA) ') ' num2str(round(obj.arenaVars.dt/mean(t_ar(1:ti)),2)) 'x\n']);
                    if ti>1
                        fprintf(char(repmat('\b',1,prev_l-2)));
                    end
                    fprintf (text);
                    prev_l = length(text);
                elseif obj.print>1 && ~mod(ti,obj.print*round(nT/100))
                    t_ar(ti)    = toc(iterT);                       % Get elapsed time
                    iterT       = tic;                              % Set new time
                    perc        = ti/nT;                            % Get iteration percentage
                    ETA         = round(mean(t_ar(1:ti))*(nT-ti));  % Calculate time remaining
                    text        = strcat(['' char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' obj.sec2time(ETA) ') ' num2str(round(obj.arenaVars.dt/mean(t_ar(1:ti)),2)) 'x\n']);
                    fprintf (text);
                end
            end
            if obj.frame_save~=1
                close(obj.movie);
            end
            close(obj.fig);
            filename = obj.moviefile;
        end
        
        function printAgents(obj,t,pos,head,collision_count)
            set(obj.fig, 'currentaxes', obj.axes);                  % Select axes
            if t == 1
                cla(obj.axes);                                       % Empty axes
                % Plot shapes
                hold on;
                obj.fh_drone = viscircles(pos(:,1:2), 0.5*obj.arenaVars.collision_range*ones(obj.arenaVars.nAgents,1), 'Color', 'black', 'LineStyle', '-', 'LineWidth',1);
                if obj.p_circ==1
                    obj.fh_circles = viscircles(pos(:,1:2), 0.5*ones(obj.arenaVars.nAgents,1)+0.5*obj.arenaVars.collision_range, 'Color', 'black', 'LineStyle', '--', 'LineWidth',1);
                end
                if obj.p_head==1
                    [cx,cy,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1),zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*obj.arenaVars.cam_range);
                    obj.fh_head = plot([pos(:,1) (pos(:,1)+cx)]',[pos(:,2) (pos(:,2)+cy)]','r--');
                elseif obj.p_head==2
                    [cx,cy,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1),zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*0.5*obj.arenaVars.collision_range);
                    obj.fh_head = plot([pos(:,1) (pos(:,1)+cx)]',[pos(:,2) (pos(:,2)+cy)]','r--');
                end
                if obj.p_fov==1
                    [lx,ly,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1)-0.5*obj.arenaVars.cam_fov,zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*obj.arenaVars.cam_range);
                    [rx,ry,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1)+0.5*obj.arenaVars.cam_fov,zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*obj.arenaVars.cam_range);
                    obj.fh_fovL = plot([pos(:,1) (pos(:,1)+lx)]',[pos(:,2) (pos(:,2)+ly)]','b--');
                    obj.fh_fovR = plot([pos(:,1) (pos(:,1)+rx)]',[pos(:,2) (pos(:,2)+ry)]','g--');
                elseif obj.p_fov==2
                    [lx,ly,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1)-0.5*obj.arenaVars.cam_fov,zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*0.5*obj.arenaVars.collision_range);
                    [rx,ry,~] = sph2cart(head(:,1)+obj.arenaVars.cam_dir(1)+0.5*obj.arenaVars.cam_fov,zeros(obj.arenaVars.nAgents,1),ones(obj.arenaVars.nAgents,1)*0.5*obj.arenaVars.collision_range);
                    obj.fh_fovL = plot([pos(:,1) (pos(:,1)+lx)]',[pos(:,2) (pos(:,2)+ly)]','b--');
                    obj.fh_fovR = plot([pos(:,1) (pos(:,1)+rx)]',[pos(:,2) (pos(:,2)+ry)]','g--');
                end
                if obj.p_label==1
                    labels = cellstr(int2str((1:obj.arenaVars.nAgents)'));
                    obj.fh_label = text(pos(:,1),pos(:,2),labels,'Color','k','FontSize',7);
                end
                obj.fh_cpos = plot(obj.arenaVars.c_pos(t,1),obj.arenaVars.c_pos(t,2),'x');
                hold off;
            else
                % Update shapes
                ppA_d = length(obj.fh_drone.Children(1).XData)/obj.arenaVars.nAgents;
                if obj.p_circ == 1
                    ppA_c   = length(obj.fh_circles.Children(1).XData)/obj.arenaVars.nAgents;
                end
                for i=1:obj.arenaVars.nAgents
                    % Clear old collision warning if exists
                    if isa(obj.fh_collisions{i},'handle')
                        delete(obj.fh_collisions{i});
                    end
                    % Show new collision warning if colliding
                    if obj.arenaVars.collisions(t,i) > 0
                        hold on;
                        obj.fh_collisions{i} = viscircles(pos(i,1:2), 0.5+0.5*obj.arenaVars.collision_range, 'Color', 'red', 'LineStyle', '-', 'LineWidth',1);
                        hold off;
                    end
                    % Update remaining shapes
                    aStart = ((i-1)*ppA_d+1):((i-1)*ppA_d+ppA_d);
                    obj.fh_drone.Children(1).XData(aStart) = obj.fh_drone.Children(1).XData(aStart) + (pos(i,1) - mean(obj.fh_drone.Children(1).XData(aStart(1:end-1))));
                    obj.fh_drone.Children(1).YData(aStart) = obj.fh_drone.Children(1).YData(aStart) + (pos(i,2) - mean(obj.fh_drone.Children(1).YData(aStart(1:end-1))));
                    if obj.p_circ==1
                        aStart  = ((i-1)*ppA_c+1):((i-1)*ppA_c+ppA_c);
                        obj.fh_circles.Children(1).XData(aStart) = obj.fh_circles.Children(1).XData(aStart) + (pos(i,1) - mean(obj.fh_circles.Children(1).XData(aStart(1:end-1))));
                        obj.fh_circles.Children(1).YData(aStart) = obj.fh_circles.Children(1).YData(aStart) + (pos(i,2) - mean(obj.fh_circles.Children(1).YData(aStart(1:end-1))));
                    end
                    if obj.p_head==1
                        [cx,cy,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1),0,obj.arenaVars.cam_range);
                        obj.fh_head(i).XData = [pos(i,1) (pos(i,1)+cx)];
                        obj.fh_head(i).YData = [pos(i,2) (pos(i,2)+cy)];
                    elseif obj.p_head==2
                        [cx,cy,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1),0,0.5*obj.arenaVars.collision_range);
                        obj.fh_head(i).XData = [pos(i,1) (pos(i,1)+cx)];
                        obj.fh_head(i).YData = [pos(i,2) (pos(i,2)+cy)];
                    end
                     if isa(obj.arenaVars.agents{i},'neuralnetAgent') && obj.p_head>0
                        obj.fh_head(i).Color = 'blue';
                    end
                    if obj.p_fov==1
                        [lx,ly,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1)-0.5*obj.arenaVars.cam_fov,0,obj.arenaVars.cam_range);
                        [rx,ry,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1)+0.5*obj.arenaVars.cam_fov,0,obj.arenaVars.cam_range);
                        obj.fh_fovL(i).XData = [pos(i,1) (pos(i,1)+lx)];
                        obj.fh_fovL(i).YData = [pos(i,2) (pos(i,2)+ly)];
                        obj.fh_fovR(i).XData = [pos(i,1) (pos(i,1)+rx)];
                        obj.fh_fovR(i).YData = [pos(i,2) (pos(i,2)+ry)];
                    elseif obj.p_fov==2
                        [lx,ly,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1)-0.5*obj.arenaVars.cam_fov,0,0.5*obj.arenaVars.collision_range);
                        [rx,ry,~] = sph2cart(head(i,1)+obj.arenaVars.cam_dir(1)+0.5*obj.arenaVars.cam_fov,0,0.5*obj.arenaVars.collision_range);
                        obj.fh_fovL(i).XData = [pos(i,1) (pos(i,1)+lx)];
                        obj.fh_fovL(i).YData = [pos(i,2) (pos(i,2)+ly)];
                        obj.fh_fovR(i).XData = [pos(i,1) (pos(i,1)+rx)];
                        obj.fh_fovR(i).YData = [pos(i,2) (pos(i,2)+ry)];
                    end
                    if obj.p_label==1
                        obj.fh_label(i).Position(1:2) = [pos(i,1) pos(i,2)];
                    end
                end
                obj.fh_cpos.XData = obj.arenaVars.c_pos(t,1);
                obj.fh_cpos.YData = obj.arenaVars.c_pos(t,2);
            end
            % Set axes
            if obj.p_mov_axe == 1
                axis equal tight;
            end
            if length(obj.p_axe_lim) == 2;
                limX = xlim;
                limY = ylim;
                axis([  min(limX(1),-0.5*obj.p_axe_lim(1)) max(limX(2),0.5*obj.p_axe_lim(1))...
                    min(limY(1),-0.5*obj.p_axe_lim(2)) max(limY(2),0.5*obj.p_axe_lim(2))]);
            elseif length(obj.p_axe_lim) == 4;
                limX = xlim;
                limY = ylim;
                axis([  min(limX(1),obj.p_axe_lim(1)) max(limX(2),obj.p_axe_lim(2))...
                    min(limY(1),obj.p_axe_lim(3)) max(limY(2),obj.p_axe_lim(4))]);
            end
            % Update figure table
            obj.fig_table{2}.updateTable({collision_count sprintf('%0.1f',t*obj.arenaVars.dt)}); % Update time and collisions
            drawnow;
            % Save frame or add to movie
            if obj.frame_save==1
                print(obj.fig,'-dsvg',strcat(['./movies_' obj.diskType '/frames/' obj.name '_frame-' num2str(t*obj.arenaVars.dt) '.svg'])); %#ok Save every frame as EPS file
                %print(obj.fig,'-djpeg',strcat(['./movies_' obj.diskType '/frames/' obj.name '_frame-' num2str(t*obj.arenaVars.dt) '.jpg']),'-r0'); % Save every frame as JPG file
                %saveas(obj.fig,strcat(['./movies_' obj.diskType '/frames/' obj.name '_frame-' num2str(t*obj.arenaVars.dt) '.png'])); % Save every frame as PNG file
            else
                F = getframe(obj.fig);
                writeVideo(obj.movie,F);
            end
            if obj.showFig==0
                set(obj.fig,'Visible','off'); 
            end
        end
        
        function string = sec2time(~,ETA)
            % Convert e.g. 3661sec to 01:01:01
            ETA_h   = floor(ETA/3600);
            ETA_m   = floor((ETA-3600*ETA_h)/60);
            ETA_s   = ETA - 3600*ETA_h - 60*ETA_m;
            if ETA_h <10
                ETA_h = strcat(['0' num2str(ETA_h)]);
            else
                ETA_h = num2str(ETA_h);
            end
            if ETA_m <10
                ETA_m = strcat(['0' num2str(ETA_m)]);
            else
                ETA_m = num2str(ETA_m);
            end
            if ETA_s <10
                ETA_s = strcat(['0' num2str(ETA_s)]);
            else
                ETA_s = num2str(ETA_s);
            end
            string  = strcat([ETA_h ':' ETA_m ':' ETA_s]);
        end
    end
    
end

