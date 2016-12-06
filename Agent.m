classdef Agent < handle
    %AGENT Agent class modelling an agent
    %   Agent class modelling an agent
    
    properties
        id;                                         % UID of agent
        % Agent properties
        collision_range     = 0.3;                  % Collision_range [m]
        seperation_range    = 1.5;                  % Seperation_range [m]
        v_max               = 1;                    % Max v in m/s
        th_max              = 200/180*pi();         % Max theta in rad/s
        t_mem               = 20;                   % Memory [timesteps]
        % Camera properties
        cam_dir             = [0 -30/180*pi()];     % Camera direction [radian yaw, radian pitch]
        cam_fov             = 152/180*pi();         % Camera FOV [radian]
        cam_range           = 4;                    % Camera range [m]
        cam_acc             = 0.95;                 % Accuracy of Camera
        % Non-optional properties
        neighbours          = {};                   % Structure to save neighbours
        u_d_decom           = {};                   % Structure to save decomposed v_d into seperate signals
        pos                 = [];                   % Matrix containing current position ([x y z])
        heading             = [];                   % Matrix containing current heading ([yaw pitch])
        vel                 = [];
        genome              = [];
        % Cost
        collisions          = 0;                    % Matrix containing collisions (t,id)
        vel_cost            = 0;
        dist_cost           = 0;
        % Placeholders
        net                 = @(x) 0;               % Placeholder for simpleNN network
        % Arena variables
        dt                  = 0;
        t                   = 0;
        T                   = 0;
        swarmMode           = 2;
        swarmSize           = 1;
        c_pos               = [0 0 10];
        circle_packing_radius;
        prev_vd             = 0;
        field_type          = 'bucket';
        field_varargin;
        c_fun               = 0;
    end
    
    methods
        function obj = Agent(arenaC,id,pos,head)
            obj.T                       = arenaC.T;
            obj.dt                      = arenaC.dt;
            obj.swarmMode               = arenaC.swarmMode;
            obj.swarmSize               = arenaC.nAgents;
            obj.circle_packing_radius   = arenaC.circle_packing_radius;
            obj.id                      = id;
            % Used to initialize the Agent and create matrices
            obj.pos                     = pos;
            obj.vel                     = [0 0 0];
            obj.heading                 = head;
            obj.collisions              = zeros(obj.T/obj.dt+1,1);
            obj.neighbours              = cell(obj.T/obj.dt,1);
        end
        
        function [v_d,theta,phi] = Update(obj,t,pos,heading,vel,neighbours,agent_positions,agent_distances)
            obj.t                   = t;
            obj.pos                 = pos;
            if isa(obj.c_fun,'function_handle')
                obj.c_pos               = feval(obj.c_fun, t * obj.dt);
            end
            obj.vel                 = vel;
            obj.heading             = heading;
            obj.neighbours{t}       = obj.buildNeighbourMatrix(neighbours, agent_positions, agent_distances);                     % Using detected neighbours build the matrix
            [v_d,g_d]               = obj.calculate_vd(obj.neighbours{t}(:,3:6));
            v_dhat                  = v_d/norm(v_d);
            if obj.swarmMode == 1
                phi                     = atan2(v_dhat(2),v_dhat(1));                           % Yaw angle of v_d
                theta                   = atan2(v_dhat(3),v_d_n);                               % Pitch angle of v_d
            else
                phi                     = atan2(g_d(2),g_d(1));                           % Yaw angle of g_d
                theta                   = atan2(g_d(3),0);
            end
            obj.vel_cost            = obj.vel_cost + (sqrt(v_d(1)^2 + v_d(2)^2) / sqrt(g_d(1)^2 + g_d(2)^2) - 1).^2;                             % Apply agent dynamics to desired velocity
        end
        
        function m_neighbours = buildNeighbourMatrix(obj,neighbours,agent_positions, agent_distances)
            m_neighbours    = zeros(0,6);
            newStart        = 0;
            if obj.t > 1
                prev_neighbours = obj.neighbours{obj.t-1};
                if~isempty(prev_neighbours)
                    m_neighbours    = prev_neighbours(prev_neighbours(:,2) > (obj.t-obj.t_mem),:);                           % Select neighbours from memory which haven't degraded
                    m_neighbours    = m_neighbours(logical(sum(m_neighbours(:,1)*ones(size(neighbours))==ones(size(m_neighbours(:,1)))*neighbours,2)==0),:);   % Only keep which we cant see
                    newStart        = size(m_neighbours,1);
                end
            end
            if ~isempty(neighbours)
                m_neighbours    = [m_neighbours; zeros(length(neighbours),6)];
                dist_noise      = randn(length(neighbours),1) .* (1 - obj.cam_acc) .* agent_distances';
                pos_noise       = dist_noise .* (obj.pos' - agent_positions(neighbours,:)) ./ agent_distances';
                for j = 1:length(neighbours)
                    j_pos                       = agent_positions(neighbours(j),:)' - pos_noise(j,:)';
                    m_neighbours(newStart+j,:)  = [neighbours(j) obj.t j_pos' agent_distances(j)+dist_noise(j)]; % Not yet in memory - insert
                end
            end
        end
        
        function [v_d,g_i] = calculate_vd(obj, neighbours)
            L_i = [0 0 0];        % Lattice formation
            g_i = obj.global_interaction(obj.pos');
            if ~isempty(neighbours)
                q_ij    = obj.pos' - neighbours(:,1:3);       % Relative vector between agent i and j
                nMag    = neighbours(:,4);
                nDir    = q_ij ./ nMag;
                nL_i    = obj.local_interaction(nMag')';
                L_i     = sum(nL_i .* nDir,1)./length(nL_i);    % Average over nr. of agents
            end
            %u_d     = L_i + (obj.v_max - obj.genome(2) * min(obj.v_max,sqrt(L_i(1)^2+L_i(2)^2)))/obj.v_max * g_i;                                    % Sum to find u_d
            u_d     = L_i + max(0, obj.v_max - obj.genome(2) * sqrt(L_i(1)^2+L_i(2)^2))^2/(obj.v_max^2) * g_i;
            
            u_d(3)  = 0;
            u_d_n   = sqrt(u_d(1)^2 + u_d(2)^2);% + u_d(3)^2);
            if u_d_n > obj.v_max
                g_i = g_i / u_d_n * obj.v_max;
                L_i = L_i / u_d_n * obj.v_max;
                u_d = g_i + L_i;
            end
            d_i = -obj.genome(1)*(L_i+g_i - obj.prev_vd);   % Calculate dissipative energy
            v_d = u_d + d_i;
            obj.prev_vd = v_d;
        end
        
        function y = local_interaction(x)
            y       = x;
        end
        
        function X = getAgentFunction(obj,x)
            X       = zeros(size(x));
            for r=1:length(x)
                X(r)    = obj.local_interaction(x(r));
            end
        end
        
        function g = global_interaction(obj,x)
            x = x - obj.c_pos;
            switch obj.field_type
                case 'circle'
                g       = obj.circleField(x, obj.field_varargin); 
                case 'bucket'
                g       = obj.bucketField(x, obj.field_varargin);
                case 'point'
                g       = obj.pointField(x, obj.field_varargin);
                otherwise
                    obj.field_type      = 'bucket';
                    obj.field_varargin  = cell(obj.seperation_range + obj.collision_range,0.9*obj.v_max,0.1*obj.v_max);
                    g                   = global_interaction(x, obj.field_varargin);
            end
        end
        
        function g_i = pointField(~, pos, inputArgs)
            % pos, v_max
            g_i             = inputArgs{1} * [-pos(1) -pos(2) 0]./sqrt(pos(1)^2+pos(2)^2);
        end
        
        function g_i = bucketField(obj, pos, inputArgs)
            % pos, seperation_distance, v_max, v_min
            pos_n           = sqrt(pos(1)^2+pos(2)^2);
            bucket_radius   = obj.circle_packing_radius(obj.swarmSize) * inputArgs{1};
            g_in            = (1 - 1 / (1 + exp(6 / (1 * bucket_radius) * (pos_n - (1 * bucket_radius) )))) * inputArgs{2};
            g_id            = [-pos(1) -pos(2) 0]./pos_n;
            g_i             = max(inputArgs{3},min(inputArgs{2},g_in)) * g_id;
        end
        
        function g_i = circleField(~, pos, inputArgs)
            % pos, radius, v_max, direction, band_width_gain, spiral_gain
            pos_n           = sqrt(pos(1)^2+pos(2)^2);
            if length(inputArgs)>=4
                band_width_gain = inputArgs{4};
            else
                band_width_gain = 0.25;
            end
            if length(inputArgs)>=5
                spiral_gain = inputArgs{5};
            else
                spiral_gain = 6;
            end
            g_i             = inputArgs{2} * min(0.9,(0.5 + band_width_gain*(pos_n-inputArgs{1})^2)) * rotz(inputArgs{3} * (90 + min(90,max(-90,spiral_gain*(inputArgs{1} - pos_n))))) * 1/pos_n * [-pos(1); -pos(2); 0];
            g_i             = g_i';
        end
        
        function F = plotGlobalAttraction(obj,x_arr,y_arr,varargin)
            resfac      = 20;
            [x,y]       = meshgrid(x_arr,y_arr);
            x           = reshape(x,[],1);
            y           = reshape(y,[],1);
            if ~isempty(varargin)
                for i=1:length(varargin)
                    if isa(varargin{i},'function_handle')
                        
                    elseif isa(varargin{i},'double')
                        if length(varargin{i})==3
                            obj.c_pos  = varargin{i};
                        end
                    elseif isa(varargin{i},'logical')
                        if varargin{i}
                            F       = figure();
                        else
                            F       = false;
                        end
                    end
                end
                    
            end
            Rij         = [x y zeros(size(x))];
            g_i         = zeros(length(Rij),3);
            for i=1:length(Rij)
                g_i(i,:)    = obj.global_interaction(Rij(i,:));
            end
            vNorm   = reshape(sqrt(g_i(:,1).^2+g_i(:,2).^2+g_i(:,3).^2),length(y_arr),length(x_arr));
            u       = reshape(g_i(:,1),length(y_arr),length(x_arr))./vNorm;
            v       = reshape(g_i(:,2),length(y_arr),length(x_arr))./vNorm;
            w       = reshape(g_i(:,3),length(y_arr),length(x_arr))./vNorm;
            if ~exist('F','var')
                F = figure();
            end
            hold on;
            surf(x_arr,y_arr, vNorm,'EdgeColor','none','LineStyle','none');
            quiver3(x_arr(1:resfac:end),y_arr(1:resfac:end), vNorm(1:resfac:end,1:resfac:end),u(1:resfac:end,1:resfac:end),v(1:resfac:end,1:resfac:end),w(1:resfac:end,1:resfac:end),0,'Color','k');
            hold off;
            axis equal tight;
            h       = colorbar();
            title('Global attractor - g_i(r_{ij})');
            xlabel('x position [m]');
            ylabel('y position [m]');
            zlabel('Velocity [m/s]');
            ylabel(h,'Velocity [m/s]');
        end
    end
end