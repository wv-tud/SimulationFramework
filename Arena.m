classdef Arena < handle
    %ARENA Arena class modelling an arena
    %   Arena class modelling an arena
    
    properties
        % Simulation parameters
        simId           = 0;               % Simulation identifier
        name            = 'defaultArena';  % Arena name (used for movie file)
        typeName        = 'default';       % Defines arena type name
        T               = 30;              % Simulation time
        dt              = 0.1;             % timestep
        gustVelocity    = 1.0;             % m/s
        nAgents         = 5;               % Number of agents to be spawned
        nnAgents        = 0;               % Number of NN agents of agents to be spawned
        polyAgents      = 0;               % Number of poly Agents
        sinusoidAgents  = 0;               % Number of sinusoid Agents
        swarmMode       = 2;               % 1= look where you go, 2= look towards global
        init            = 'random';        % initialisation procedure ['random','rect','square']
        boc             = 0;               % Break simulation on impact
        size            = [10 10];         % Size of spawn arena [x y][m]
        diskType        = 'hdd';           % Check filenames existing on HDD or SSD
        indiRate        = 512;
        field           = struct('name','default_bucket','type','bucket');
        % Boolean options
        addAgents       = false;
        moving_axes     = false;
        save            = false;           % save output to .mat file
        print           = true;            % Print waitbar + ETA
        % Counters
        t               = 0;               % current time
        % Placeholders
        collisions      = [];              % Counts the colissions
        agents          = {};              % Agent struct
        a_positions     = [];              % Vector containing agents positions
        a_headings      = [];              % Vector containing agents headings
        a_velocities    = [];
        noise_u         = [];
        noise_v         = [];
        distance_cost   = [];
        seperation_cost = [];
        circle_packing_radius = [];        % Placeholder for bucket diameter multipliers calculated on init
        agent_conf      = struct();        % Additional agent configuration
    end
    
    methods
        function obj = Arena(name)
            % Used to initialize the Arena when new instance is created
            obj.name    = name;
            obj.simId   = name;
        end
        
        function Simulate(obj,varargin)
            obj.circle_packing_radius = sqrt((obj.nAgents*(obj.agent_conf.seperation_range+obj.agent_conf.collision_range)^2*sqrt(3)/2)/pi());
            nT                  = obj.T/obj.dt; % Number of timestep iterations
            % Preallocate arrays
            obj.collisions      = zeros(nT,obj.nAgents);
            obj.a_positions     = zeros(nT+1,obj.nAgents,3);
            obj.a_headings      = zeros(nT+1,obj.nAgents,2);
            obj.a_velocities    = zeros(nT+1,obj.nAgents,3);
            % Start simulation
            obj.initAgents(obj.nAgents);        % Create all agents
            obj.initSimulation();               % Prepare movie,figure and variables
            t_ar                = zeros(nT,1);  % Array containing timestep calculation time
            obj.distance_cost   = zeros(nT,1);
            obj.seperation_cost = zeros(nT,1);
            if obj.print>0
                iterT = tic;  % Start timer for first iteration
            end
            for ti = 1:nT
                obj.t               = ti;
                [neighbours,dAbs]      = obj.detectNeighbours(ti);     % Calculate agents in each others FOV cone & save collisions
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
                    a_n_mat                       = find(neighbours(i,:)>0);
                    distances                     = dAbs(i,a_n_mat);
                    [v_d(i,:),theta(i),phi(i)]    = obj.agents{i}.Update(obj.t,reshape(obj.a_positions(ti,i,:),[3 1]), reshape(obj.a_headings(ti,i,:),[2 1]), reshape(obj.a_velocities(ti,i,:),[3 1]), a_n_mat, reshape(obj.a_positions(ti,:,:),[obj.nAgents 3]), distances);
                end
                v_d             = obj.indiGuidance(v_d);
                obj.a_positions(ti+1,:,:)   = reshape(obj.a_positions(ti,:,:),[obj.nAgents 3]) + v_d;% + [obj.noise_u(:,obj.t) obj.noise_v(:,obj.t) zeros(obj.nAgents,1)];
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
            nrFields            = length(obj.field);
            fieldAgents         = obj.chunkSplit(cAgents+1:obj.nAgents, nrFields);
            agentFieldCentres   = [];
            agentsPerField      = sum(fieldAgents'>0);
            for f=1:nrFields
                agentFieldCentres(end+1:end+agentsPerField(f),1:3) = repmat(obj.field(f).c_pos,agentsPerField(f),1);
            end
            if strcmp(obj.init,'random')
                pos         = unifrnd(-0.5,0.5,newAgents,2);
                pos(:,1)    = pos(:,1)*obj.size(1) + agentFieldCentres(:,1);
                pos(:,2)    = pos(:,2)*obj.size(2) + agentFieldCentres(:,2);
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
                pos     = [reshape(x,numel(x),1)-0.5*obj.size(1)+agentFieldCentres(:,1) reshape(y,numel(y),1)-0.5*obj.size(2)+agentFieldCentres(:,2)];
                head    = unifrnd(0,2*pi(),newAgents,1);
            elseif strcmp(obj.init,'square')
                xs      = ceil(sqrt(newAgents));
                ys      = xs;
                [x,y]   = meshgrid((obj.size(1)/xs:obj.size(1)/xs:obj.size(1))-0.5*obj.size(1)/xs,(obj.size(2)/ys:obj.size(2)/ys:obj.size(2))-0.5*obj.size(2)/ys);
                pos     = [reshape(x,numel(x),1)-0.5*obj.size(1) reshape(y,numel(y),1)-0.5*obj.size(2)];
                pos     = pos(1:newAgents,:)+agentFieldCentres(:,1:2);
                head    = unifrnd(0,2*pi(),newAgents,1);
            end
            for i=1:obj.sinusoidAgents
                obj.agents{cAgents+i} = Agent_sinusoid(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.sinusoidAgents+1):(obj.sinusoidAgents+obj.polyAgents)
                obj.agents{cAgents+i} = Agent_polynomial(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.sinusoidAgents+obj.polyAgents+1):(obj.sinusoidAgents+obj.polyAgents+obj.nnAgents)
                obj.agents{cAgents+i} = Agent_simpleNN(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            for i=(obj.polyAgents+obj.sinusoidAgents+obj.nnAgents+obj.nnAgents+1):newAgents
                obj.agents{cAgents+i} = Agent_pinciroli(obj,cAgents+i,[pos(i,:) 10],[head(i) 0]); % Add agent
                obj.agents{cAgents+i} = obj.mergeStruct(obj.agents{cAgents+i},obj.agent_conf); % Pass agent config
            end
            obj.nAgents = cAgents + newAgents;
            obj.a_positions(1,:,:)  = [pos  10*ones(newAgents,1)];
            obj.a_headings(1,:,:)   = [head zeros(newAgents,1)];
            if strcmp(obj.init,'random') % Find collisions and resolve them before continuing
                collisionFree = 0;
                k = 1;
                u = 1;
                tries = 0;
                while collisionFree == 0
                    [~,dAbs]    = obj.detectNeighbours(1);
                    [i,~]       = find(dAbs<(obj.agents{1}.collision_range+k*obj.agents{1}.seperation_range));
                    if isempty(i)
                        collisionFree  = 1;
                        continue;
                    end
                    tries = tries + 1;
                    if mod(tries,2) == 0
                        if k > 0.8
                            k = 0.99 * k;   % First decrease the required seperation to 80%
                        else
                            u = 1.01 * u;   % If that didn't work then let's try to increase the size
                        end
                    end
                    id = unique(i);
                    for i=1:length(id)
                        new_pos                         = unifrnd(-0.5*u,0.5*u,1,2);
                        new_pos(:,1)                    = new_pos(:,1)*obj.size(1) + agentFieldCentres(id(i),1);
                        new_pos(:,2)                    = new_pos(:,2)*obj.size(2) + agentFieldCentres(id(i),2);
                        obj.agents{id(i)}.pos(1,1:2)    = new_pos;
                        obj.a_positions(1,id(i),1:2)    = new_pos;
                    end
                end
                obj.size(1) = max(max(obj.size(1),max(max(abs(squeeze(obj.a_positions(1,:,1)))))),max(obj.size(2),max(max(abs(squeeze(obj.a_positions(1,:,2)))))));
                obj.size(2) = obj.size(1);
                %obj.size = obj.size .* u;   % Save the (optionally) updated size
            end
            % Pass the relevant field characteristics to each agent
            for i=1:nrFields
                for a=fieldAgents(i,fieldAgents(i,:)>0)
                   obj.agents{a}.field_type     = obj.field(i).type;
                   obj.agents{a}.field_varargin = struct2cell(obj.field(i).varargin);
                   if isfield(obj.field(i),'c_fun')
                       obj.agents{a}.c_fun = obj.field(i).c_fun;
                   else
                       if isfield(obj.field(i),'c_pos')
                            obj.agents{a}.c_pos = obj.field(i).c_pos;
                       end
                   end
                   switch obj.field(i).type
                       case 'circle'
                           obj.agents{a}.fieldFunction = @(x, field_varargin) obj.agents{a}.circleField(x, field_varargin);
                       case 'bucket'
                           obj.agents{a}.fieldFunction = @(x, field_varargin) obj.agents{a}.bucketField(x, field_varargin);
                       case 'point'
                           obj.agents{a}.fieldFunction = @(x, field_varargin) obj.agents{a}.pointField(x, field_varargin);
                   end
               end
            end
        end
        
        function b = chunkSplit(~, a, n)
            ceilR       = ceil(length(a)/n);
            modR        = ceilR * n - length(a);
            if modR == 0
                b           = reshape(a(1:end),ceilR,[]);
            else
                b           = reshape(a(1:ceilR*(n-1)),[],n-1);
                b(:,end+1)     = zeros(1,ceilR);
                b(1:ceilR-modR,end) = a(ceilR*(n-1)+1:end); 
            end
            b =b';
        end
        
        function initSimulation(obj)
            % Naming
            obj.name    =  sprintf('%s-%0.0f-%s-%0.0fs-%0.1fms-%0.1fm-%0.0fm2-v1',obj.typeName, obj.nAgents, obj.field(1).name, obj.T,obj.agents{1}.v_max,obj.agents{1}.seperation_range,obj.size(1)*obj.size(2)*4); % Generate name based on current arena settings
            
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
            sigma_w     = 0.1 * obj.gustVelocity; % W20 = 15 knts = 7.7 m/s
            sigma_u     = sigma_w / (0.177 + 0.000823 * 10)^(0.4);
            sigma_v     = sigma_u;
            V           = 0.5 * obj.agents{1}.v_max;
            Lu          = 1 / (0.177 + 0.000823*1)^(1.2);
            Lv          = Lu; % 0.5 * Lu?
            indiRuns    = round(obj.indiRate * obj.dt);
            gt          = 0:obj.dt/indiRuns:(obj.T);
            wu          = randn(obj.nAgents, length(gt))./sqrt(obj.dt/round(obj.indiRate*obj.dt));
            wv          = randn(obj.nAgents, length(gt))./sqrt(obj.dt/round(obj.indiRate*obj.dt));
            C           = [ 1 0 ];
            D           = 0;
            % Noise u
            obj.noise_u = zeros(obj.nAgents,length(gt));
            rat         = V / Lu;
            A           = [ 0 1; -rat^2 -2*rat ];
            B           = sigma_u * [ sqrt(3*rat); (1- 2 * sqrt(3)) * sqrt((rat^3)) ];
            for i = 1:obj.nAgents
                obj.noise_u(i,:)  = lsim(A, B, C, D, wu(i,:), gt);
            end
            % Noise v
            obj.noise_v = zeros(obj.nAgents, length(gt));
            rat         = V / Lv;
            A           = [ 0 1; -rat^2 -2*rat ];
            B           = sigma_v * [ sqrt(3*rat); (1- 2 * sqrt(3)) * sqrt((rat^3)) ];
            for i = 1:obj.nAgents
                obj.noise_v(i,:)  = lsim(A, B, C, D, wv(i,:), gt);
            end
%             figure();
%             hold all;
%             plot(gt,obj.noise_u);
%             plot(gt,obj.noise_v);
%             hold off;
%             pause
        end
        
        function [neighbours,dAbs] = detectNeighbours(obj,t)
            headMap     = reshape(meshgrid(obj.a_headings(t,:,1),ones(obj.nAgents,1)),obj.nAgents,obj.nAgents,1);
            posMap      = reshape(meshgrid(obj.a_positions(t,:,:),ones(obj.nAgents,1)),obj.nAgents,obj.nAgents,3);
            rij_x  = shiftdim(reshape(posMap(:,:,1),[obj.nAgents obj.nAgents])',2)-(posMap(:,:,1)); % Create rij matrix rij(i,j,[x y z])
            rij_y  = shiftdim(reshape(posMap(:,:,2),[obj.nAgents obj.nAgents])',2)-(posMap(:,:,2));
            rij_z  = 0; %shiftdim(reshape(posMap(:,:,3),[obj.nAgents obj.nAgents])',2)-(posMap(:,:,3));
            dAbs        = sqrt(rij_x.^2+rij_y.^2+rij_z.^2) + diag(ones(1,obj.nAgents))*obj.agents{1}.cam_range; % Calculate abs distance (i==j can't see itself)
            if t>1
                col_mat     = find(sum(dAbs < obj.agents{1}.collision_range,2)>0); % Detect and save collisions
                for i=1:length(col_mat)
                    obj.collisions(t,col_mat(i))            = 1; % Count total nr of collisions
                end
            end
            %obj.distance_cost = obj.distance_cost + sum(sum(abs(((obj.agents{1}.collision_range + obj.agents{1}.seperation_range) ./ dAbs).^2 - 1) .* (cumsum(diag(ones(1,obj.nAgents)))' - diag(ones(1,obj.nAgents)))));
            % Alpha-lattice deformation: https://pdfs.semanticscholar.org/ccfb/dd5c796bb485effe8a035686d785e8306ff4.pdf
            if obj.t > 0
                tmp_cost        = 0;
                tmp_sep_cost    = 0;
                sigma           = (obj.agents{1}.collision_range + obj.agents{1}.seperation_range);
                dCostIndices    = dAbs < obj.agents{1}.cam_range;
                for i=1:obj.nAgents
                    dCostMat        = dAbs(i,dCostIndices(i,:));
                    dCostnAgents    = length(dCostMat);
                    if dCostnAgents == 0
                        tmp_sep_cost    = tmp_sep_cost + 1;
                    else
                        tmp_cost        = tmp_cost + 1/(dCostnAgents+1) * sum((dCostMat-sigma).^2);
                    end
                end
                obj.distance_cost(obj.t)    = tmp_cost;
                obj.seperation_cost(obj.t)  = tmp_sep_cost;
            end
            angles      = obj.smallAngle(headMap + obj.agents{1}.cam_dir(1) - atan2(reshape(rij_y,[obj.nAgents obj.nAgents]),reshape(rij_x,[obj.nAgents obj.nAgents]))); % Calculate angles
            %anglesZ    = zeros(obj.nAgents,obj.nAgents,1) - obj.agents{1}.cam_dir(2) + atan2(squeeze(rij_z),sqrt(squeeze(rij_x.^2)+squeeze(rij_y.^2))); %calculates the pitch angle
            neighbours  = (dAbs < obj.agents{1}.cam_range & abs(angles) < 0.5*obj.agents{1}.cam_fov); %& abs(anglesZ) < 0.5*obj.agents{1}.cam_fov); % Save neighbours based on selection ||rij|| > cam_range && abs(angle(rij,cam_dir)) < 1/2*FOV
            neighbours  = neighbours';
        end
        
        function pos_update = indiGuidance(obj, sp)
            guidance_indi_pos_gain      = 0.5;
            guidance_indi_speed_gain    = 1.8;
            sp                          = 1/guidance_indi_pos_gain * sp;
            indiRuns                    = round(obj.indiRate*obj.dt);
            pos_update                  = zeros(obj.nAgents,3);
            tmpVel                      = reshape(obj.a_velocities(max(obj.t-1,1),:,:),[obj.nAgents 3]);
            if obj.t > 1
                noise_x  = obj.noise_u(:,(obj.t-1)*(indiRuns));
                noise_y  = obj.noise_v(:,(obj.t-1)*(indiRuns));
            else
                noise_x = 0;
                noise_y = 0;
            end
            for i=1:indiRuns
                pos_x_err   = sp(:,1) - pos_update(:,1);
                pos_y_err   = sp(:,2) - pos_update(:,2);
                %pos_z_err  = sp(:,3) - pos_update(:,3);
                speed_sp_x  = pos_x_err .* guidance_indi_pos_gain;
                speed_sp_y  = pos_y_err .* guidance_indi_pos_gain;
                %speed_sp_z = pos_z_err * guidance_indi_pos_gain;
                sp_accel_x  = (speed_sp_x - tmpVel(:,1)) .* guidance_indi_speed_gain;
                sp_accel_y  = (speed_sp_y - tmpVel(:,2)) .* guidance_indi_speed_gain;
                %sp_accel_z = (speed_sp_z - tmpVel(:,3)) ,* guidance_indi_speed_gain;                
                sp_accel_n  = sqrt(sp_accel_x.^2+sp_accel_y.^2);
                sp_accel_x  = sp_accel_x .* min(sp_accel_n, 6) ./ sp_accel_n;
                sp_accel_y  = sp_accel_y .* min(sp_accel_n, 6) ./ sp_accel_n;
                prev_noise_x    = noise_x;
                prev_noise_y    = noise_y;
                noise_x         = obj.noise_u(:,(obj.t-1)*(indiRuns)+i);
                noise_y         = obj.noise_v(:,(obj.t-1)*(indiRuns)+i);
                tmpVel(:,1) = tmpVel(:,1) + (noise_x - prev_noise_x) + obj.dt .* 1./indiRuns .* sp_accel_x;
                tmpVel(:,2) = tmpVel(:,2) + (noise_y - prev_noise_y) + obj.dt .* 1./indiRuns .* sp_accel_y;
                %tmpVel(:,3)= tmpVel + sp_accel_z;
                pos_update  = pos_update + obj.dt .* 1./indiRuns .* tmpVel;
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