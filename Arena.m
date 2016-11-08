classdef Arena < handle
    %ARENA Arena class modelling an arena
    %   Arena class modelling an arena
    
    properties
        % Simulation parameters
        name         = 'defaultArena';  % Arena name (used for movie file)
        T            = 30;              % Simulation time
        dt           = 0.1;             % timestep
        gustVelocity = 0.1;
        addAgents    = 0;
        nAgents      = 5;               % Number of agents to be spawned
        nnAgents     = 0;               % Number of NN agents of agents to be spawned
        polyAgents   = 0;               % Number of poly Agents
        sinusoidAgents = 0;             % Number of sinusoid Agents
        swarmMode    = 2;               % 1= look where you go, 2= look towards global
        init         = 'random';        % initialisation procedure ['random','rect','square']
        boc          = 0;               % Break simulation on impact    
        size         = [10 10];         % Size of spawn arena [x y][m]
        c_fun        = @(t) [0 0 10];   % Centre point function
        diskType     = 'hdd';           % Check filenames existing on HDD or SSD
        % Simulation options
        save         = 0;               % save output to .mat file
        print        = 1;               % Print waitbar + ETA
        agent_conf   = struct();        % Additional agent configuration
        % Simulation non-optional parameters
        agents       = {};              % Agent struct
        a_positions  = [];              % Vector containing agents positions
        a_headings   = [];              % Vector containing agents headings
        a_velocities = [];
        t            = 0;               % current time
        collisions   = 0;               % Counts the colissions
        c_pos;                          % desired swarm centre position
        axes;                           % Axes handle
        simId;                          % Simulation identifier
        circle_packing_radius;          % Placeholder for bucket diameter multipliers calculated on init
        net          = @(x) 0;          % Placeholder for simpleNN network
    end
    
    methods
        function obj = Arena(name)
            % Used to initialize the Arena when new instance is created
            obj.name    = name;
            obj.simId   = name;
            obj.circle_packing_radius = [ ...
                1 ...
                2 ...
                1+2/3*sqrt(3) ...
                1+sqrt(2) ...
                1+sqrt(2*(1+1/sqrt(5))) ...
                3 ...
                3 ...
                (1+1/sin(pi()/7)) ...
                1+sqrt(2*(2+sqrt(2))) ...
                3.813 ...
                1+1/sin(pi()/9) ...
                4.029 ...
                2+sqrt(5) ...
                4.328 ...
                1+sqrt(6+2/sqrt(5)+4*sqrt(1+2/sqrt(5))) ...
                4.615 ...
                4.792 ...
                1+sqrt(2)+sqrt(6) ...
                1+sqrt(2)+sqrt(6) ...
                5.122 ...
            ];
        end
        
        function Simulate(obj,varargin)
            % Start simulation
            obj.initAgents(obj.nAgents);        % Create all agents
            obj.initSimulation();               % Prepare movie,figure and variables
            t_ar    = zeros(obj.T/obj.dt,1);      % Array containing timestep calculation time
            nT      = obj.T/obj.dt;             % Number of timestep iterations
            if obj.print>0 
                iterT = tic;  % Start timer for first iteration
            end
            for ti = 1:nT
                obj.t               = ti;
                obj.c_pos(ti,:)     = feval(obj.c_fun,ti*obj.dt);   % Calculate centroid position
                [neighbours,~]      = obj.detectNeighbours(ti);     % Calculate agents in each others FOV cone & save collisions
                if obj.boc == 1
                    if sum(obj.collisions(ti,:)) > 0
                        if obj.print==1
                            fprintf('Collision occured, stopping simulation.\n');
                        end
                        return;
                    end
                end
                v_d     = zeros(obj.nAgents,3);
                theta   = zeros(obj.nAgents,1);
                phi     = zeros(obj.nAgents,1);
                for i = 1:obj.nAgents
                    [v_d(i,:),theta(i),phi(i)]    = obj.agents{i}.Update(obj.t,obj.c_pos(ti,:),squeeze(obj.a_positions(ti,i,:)), squeeze(obj.a_headings(ti,i,:)), squeeze(obj.a_velocities(ti,i,:)), find(neighbours(i,:)>0), squeeze(obj.a_positions(ti,:,:))); %#ok<FNDSB>
                end
                v_d             = obj.indiGuidance(v_d);
                noise_r         = obj.gustVelocity * randn(obj.nAgents,1);
                noise_phi       = 2*pi() * rand(obj.nAgents,1);
                noise_v         = zeros(obj.nAgents,3);
                noise_v(:,1)    = cos(noise_phi) .* noise_r;
                noise_v(:,2)    = sin(noise_phi) .* noise_r;
                obj.a_positions(ti+1,:,:)   = squeeze(obj.a_positions(ti,:,:)) + v_d + noise_v;
                obj.a_headings(ti+1,:,:)    = [phi theta];
                if obj.print==1
                    t_ar(ti)    = toc(iterT);                       % Get elapsed time
                    iterT       = tic;                              % Set new time
                    perc        = ti/nT;                            % Get iteration percentage 
                    ETA         = round(mean(t_ar(1:ti))*(nT-ti));  % Calculate time remaining
                    text        = strcat([char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' obj.sec2time(ETA) ') ' num2str(round(obj.dt/mean(t_ar(1:ti)),2)) 'x\n']);
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
                    text        = strcat(['Simulation ' obj.simId ': ' char(repmat('*',1,round(perc*20))) char(repmat('.',1,20-round(perc*20))) ' ' num2str(round(perc*100)) '%% (' obj.sec2time(ETA) ') ' num2str(round(obj.dt/mean(t_ar(1:ti)),2)) 'x\n']);
                    fprintf (text);
                end
            end
        end
        
        function initAgents(obj,newAgents)
            % Initialize agents
            if obj.addAgents == 1
                cAgents = length(obj.agents); % Agents currently in the arena
            else
                cAgents = 0;
            end
            if strcmp(obj.init,'random')
                pos         = unifrnd(-0.5,0.5,newAgents,2);
                pos(:,1)    = pos(:,1)*obj.size(1);
                pos(:,2)    = pos(:,2)*obj.size(2);
                head        = unifrnd(0,2*pi(),newAgents,1);
            elseif strcmp(obj.init,'rect')
                s = factor(newAgents);
                if length(s) == 1
                    s(2) = s(1);
                    s(1) = 1;
                end
                xs      = prod(s(1:end-1));
                ys      = s(end);
                [x,y]   = meshgrid((obj.size(1)/xs:obj.size(1)/xs:obj.size(1))-0.5*obj.size(1)/xs,(obj.size(2)/ys:obj.size(2)/ys:obj.size(2))-0.5*obj.size(2)/ys);
                pos     = [reshape(x,numel(x),1)-0.5*obj.size(1) reshape(y,numel(y),1)-0.5*obj.size(2)];
                head    = unifrnd(0,2*pi(),newAgents,1);
            elseif strcmp(obj.init,'square')
                xs      = ceil(sqrt(newAgents));
                ys      = xs;
                [x,y]   = meshgrid((obj.size(1)/xs:obj.size(1)/xs:obj.size(1))-0.5*obj.size(1)/xs,(obj.size(2)/ys:obj.size(2)/ys:obj.size(2))-0.5*obj.size(2)/ys);
                pos     = [reshape(x,numel(x),1)-0.5*obj.size(1) reshape(y,numel(y),1)-0.5*obj.size(2)];
                pos     = pos(1:newAgents,:);
                head    = unifrnd(0,2*pi(),newAgents,1);
            end
            for i=1:obj.sinusoidAgents
                obj.agents{cAgents+i} = sinusoidAgent(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.sinusoidAgents+1):(obj.sinusoidAgents+obj.polyAgents)
                obj.agents{cAgents+i} = polyAgent(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.sinusoidAgents+obj.polyAgents+1):(obj.sinusoidAgents+obj.polyAgents+obj.nnAgents)
                obj.agents{cAgents+i} = simpleNNAgent(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.polyAgents+obj.sinusoidAgents+obj.nnAgents+obj.nnAgents+1):newAgents
                obj.agents{cAgents+i} = PinciroliAgent(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            obj.nAgents = cAgents + newAgents;
            if strcmp(obj.init,'random') % Find collisions and resolve them before continuing
                collisionFree = 0;
                while collisionFree == 0
                    [~,dAbs]    = obj.detectNeighbours(1);
                    [i,~]       = find(dAbs<(obj.agents{1}.collision_range+2*obj.agents{1}.v_max*obj.dt));
                    if isempty(i)
                        obj.collisions = 0;
                        collisionFree  = 1;
                        continue;
                    end
                    id = unique(i);
                    for i=1:length(id)
                        pos                             = unifrnd(-0.5,0.5,1,2);
                        pos(:,1)                        = pos(:,1)*obj.size(1);
                        pos(:,2)                        = pos(:,2)*obj.size(2);
                        obj.agents{id(i)}.pos(1,1:2)    = pos;
                    end
                end
            end
        end
        
        function initSimulation(obj)
            % Naming
            if isa(obj,'Mission')
                obj.name    =  sprintf('Pinciroli-%0.0f-%s-%0.0fs-%0.1fms-%0.1fm-%0.0fm2-v1',obj.nAgents,obj.mission_type,obj.T,obj.agents{1}.v_max,obj.agents{1}.seperation_range,obj.size(1)*obj.size(2)*4); % Generate name based on current arena settings
            else
                obj.name    =  sprintf('Pinciroli-%0.0f-%0.0fs-%0.1fms-%0.1fm-%0.0fm2-v1',obj.nAgents,obj.T,obj.agents{1}.v_max,obj.agents{1}.seperation_range,obj.size(1)*obj.size(2)*4); % Generate name based on current arena settings
            end
            file_clear  = 0;
            version     = 1;
            while file_clear == 0 % Rename to -v1,v2,...,vN when file exists
                if exist(strcat(['./movies_' obj.diskType '/' obj.name '.mp4']), 'file') == 2
                    version = version + 1;
                    i       = 1;
                    foundV  = 0;
                    while foundV~=1
                        if obj.name(end-i) == 'v'
                            foundV = 1;
                        else
                            i = i+1;
                        end
                    end
                    obj.name = strcat([obj.name(1:end-i) num2str(version)]);
                else
                    file_clear = 1;
                end    
            end 
            % Preallocate arrays
            obj.collisions      = zeros(obj.T/obj.dt,obj.nAgents);
            obj.a_positions     = zeros(obj.T/obj.dt+1,obj.nAgents,3);
            obj.a_headings      = zeros(obj.T/obj.dt+1,obj.nAgents,2);
            obj.a_velocities    = zeros(obj.T/obj.dt+1,obj.nAgents,3);
            obj.c_pos           = zeros(obj.T/obj.dt,3);
        end
                
        function [neighbours,dAbs] = detectNeighbours(obj,t)
            if t==1
                for i=1:obj.nAgents
                    obj.a_positions(1,i,:)  = obj.agents{i}.pos(t,:);
                    obj.a_headings(1,i,:)   = obj.agents{i}.heading(t,:);
                end
            end
            headMap = reshape(meshgrid(obj.a_headings(t,:,1),ones(obj.nAgents,1)),obj.nAgents,obj.nAgents,1);
            posMap  = reshape(meshgrid(obj.a_positions(t,:,:),ones(obj.nAgents,1)),obj.nAgents,obj.nAgents,3);
            rij(:,:,1)  = shiftdim(squeeze(posMap(:,:,1))',2)-(posMap(:,:,1)); % Create rij matrix rij(i,j,[x y z])
            rij(:,:,2)  = shiftdim(squeeze(posMap(:,:,2))',2)-(posMap(:,:,2));
            rij(:,:,3)  = shiftdim(squeeze(posMap(:,:,3))',2)-(posMap(:,:,3));
            dAbs        = sqrt(rij(:,:,1).^2+rij(:,:,2).^2+rij(:,:,3).^2); % Calculate abs distance
            dAbs        = dAbs + diag(ones(1,obj.nAgents))*1.5*obj.agents{1}.cam_range; % i==j -> agent can't see itself
            if t>1
                col_mat     = find(sum(dAbs < obj.agents{1}.collision_range,2)>0); % Detect and save collisions
                for i=1:length(col_mat)
                    obj.agents{col_mat(i)}.collisions(t)    = 1; % Set which agents
                    obj.collisions(t,col_mat(i))            = 1; % Count total nr of collisions
                end
            end
            angles      = headMap + obj.agents{1}.cam_dir(1) - atan2(squeeze(rij(:,:,2)),squeeze(rij(:,:,1))); % Calculate angles
            angles      = obj.smallAngle(angles); % Transform to domain of -pi to pi
            %anglesZ     = zeros(obj.nAgents,obj.nAgents,1) - obj.agents{1}.cam_dir(2) + atan2(squeeze(rij(:,:,3)),sqrt(squeeze(rij(:,:,1).^2)+squeeze(rij(:,:,2).^2))); %calculates the pitch angle
            neighbours  = (dAbs < obj.agents{1}.cam_range & abs(angles) < 0.5*obj.agents{1}.cam_fov); %& abs(anglesZ) < 0.5*obj.agents{1}.cam_fov); % Save neighbours based on selection ||rij|| > cam_range && abs(angle(rij,cam_dir)) < 1/2*FOV
            neighbours  = neighbours';
        end
        
        function pos_update = indiGuidance(obj, sp)
            guidance_indi_pos_gain      = 0.5;
            guidance_indi_speed_gain    = 1.8;
            indiRuns                    = round(32*obj.dt);
            pos_update                  = zeros(obj.nAgents,3);
            tmpVel                      = squeeze(obj.a_velocities(max(obj.t-1,1),:,:));
            for i=1:indiRuns
                pos_x_err = sp(:,1) - pos_update(:,1);
                pos_y_err = sp(:,2) - pos_update(:,2);
                %pos_z_err = sp(:,3) - pos_update(:,3);
                speed_sp_x = pos_x_err .* guidance_indi_pos_gain;
                speed_sp_y = pos_y_err .* guidance_indi_pos_gain;
                %speed_sp_z = pos_z_err * guidance_indi_pos_gain;
                sp_accel_x = (speed_sp_x - tmpVel(:,1)) .* guidance_indi_speed_gain;
                sp_accel_y = (speed_sp_y - tmpVel(:,2)) .* guidance_indi_speed_gain;
                %sp_accel_z = (speed_sp_z - tmpVel(:,3)) ,* guidance_indi_speed_gain;
                sp_accel_x = min(0.5,max(-0.5,(sp_accel_x)));
                sp_accel_y = min(0.5,max(-0.5,(sp_accel_y)));
                tmpVel(:,1) = tmpVel(:,1) + obj.dt .* 1./indiRuns .* sp_accel_x;
                tmpVel(:,2) = tmpVel(:,2) + obj.dt .* 1./indiRuns .* sp_accel_y;
                %tmpVel = tmpVel + 1/obj.arena.dt * 1/indiRuns * sp_accel_z;
                pos_update = pos_update + tmpVel;
            end
            obj.a_velocities(obj.t,:,:) = tmpVel;
        end

        function rotmat = rotMat(~,varargin)
            angle = varargin{1};
            if nargin==3
                axis = varargin{2};
            else
                axis = 'z';
            end
            switch axis
                case 'x'
                    rotmat = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
                case 'y'
                    rotmat = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
                case 'z'
                    rotmat = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
            end
        end
        
        function small_angles = smallAngle(~,angles)
            small_angles        = angles  + floor(abs(angles./(2*pi()))).*(-2.*sign(angles))*pi();
            ipPi                = find(small_angles>pi());
            imPi                = find(small_angles<-pi());
            small_angles(ipPi)  = -2*pi() + small_angles(ipPi);
            small_angles(imPi)  = 2*pi() + small_angles(imPi);
        end
        
        function out = mergeStruct(~,varargin)
            out = varargin{1};
            if nargin > 2
                for i=2:(nargin-1)
                    % Append or overwrite
                    names = fieldnames(varargin{i});
                    for j=1:length(names)
                        out.(names{j}) = varargin{i}.(names{j});
                    end
                end
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