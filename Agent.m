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
        swarmMode           = 2;
        c_pos               = [0 0 10];
        field_type          = 'bucket';
        swing_angle         = true;
        % Camera properties
        cam_dir             = [0 -30/180*pi()];     % Camera direction [radian yaw, radian pitch]
        cam_fov             = 152/180*pi();         % Camera FOV [radian]
        cam_range           = 4;                    % Camera range [m]
        cam_acc             = 0.95;                 % Accuracy of Camera
        % Cost
        vel_cost            = 0;
        % Arena variables
        dt                  = 0;
        t                   = 0;
        T                   = 0;
        % Placeholders
        neighbours          = [];                   % Structure to save current neighbours
        prev_neighbours     = [];                   % Structure to save previous neighbours
        pos                 = [];                   % Matrix containing current position ([x y z])
        heading             = [];                   % Matrix containing current heading ([yaw pitch])
        vel                 = [];
        genome              = [];
        net                 = @(x) 0;               % Placeholder for simpleNN network
        swarmSize           = 1;
        prev_vd             = 0;
        c_fun               = 0;
        fieldFunction       = @(x) [0 0 0];
        circle_packing_radius;
        field_varargin;
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
            obj.neighbours              = zeros(0,6);
        end
        
        function [v_d,theta,phi] = Update(obj,t,pos,heading,vel,neighbours,agent_positions,agent_distances)
            obj.t                   = t;
            obj.pos                 = pos;
            if isa(obj.c_fun,'function_handle')
                obj.c_pos               = feval(obj.c_fun, t * obj.dt);
            end
            obj.vel                 = vel;
            obj.heading             = heading;
            obj.prev_neighbours     = obj.neighbours;
            obj.neighbours          = obj.buildNeighbourMatrix(neighbours, agent_positions, agent_distances');                     % Using detected neighbours build the matrix
            [v_d,g_d]               = obj.calculate_vd(obj.neighbours(:,3:6));
            v_dhat                  = v_d/sqrt(v_d(1)^2+v_d(2)^2);
            if obj.swarmMode == 1
                phi                     = atan2(v_dhat(2),v_dhat(1));                           % Yaw angle of v_d
                theta                   = 0;
                %theta                   = atan2(v_dhat(3),v_d_n);                               % Pitch angle of v_d
            else
                phi                     = atan2(g_d(2),g_d(1));                           % Yaw angle of g_d
                theta                   = 0;
                %theta                   = atan2(g_d(3),0);
            end
            if obj.swing_angle
                % Fake 180deg FOV
                phi                     = phi + 0.5 * (pi() - obj.cam_fov) * sin(2 * pi() * 1/3 * obj.t * obj.dt + obj.id);
            end
        end
        
        function m_neighbours = buildNeighbourMatrix(obj,neighbours,agent_positions, agent_distances)
            nrNeighbours    = length(neighbours);
            if~isempty(obj.prev_neighbours)
                m_neighbours    = obj.prev_neighbours(obj.prev_neighbours(:,2) > (obj.t-obj.t_mem),:);                           % Select neighbours from memory which haven't degraded
                m_neighbours    = m_neighbours(logical(-1 * sum(m_neighbours(:,1) == neighbours,2) + 1),:);
                newStart        = size(m_neighbours,1);
                m_neighbours    = [m_neighbours; zeros(nrNeighbours,6)];
            else
                newStart        = 0;
                m_neighbours    = zeros(nrNeighbours,6);
            end
            if nrNeighbours > 0
                dist_noise      = randn(nrNeighbours,1) .* (1 - obj.cam_acc) .* agent_distances;
                pos_noise       = dist_noise .* (obj.pos' - agent_positions(neighbours,:)) ./ agent_distances;
                j_pos           = agent_positions(neighbours,:) - pos_noise;
                m_neighbours(newStart+1:end,:)  = [neighbours' obj.t*ones(nrNeighbours,1) j_pos agent_distances+dist_noise];
            end
        end
        
        function [v_d,g_i] = calculate_vd(obj, neighbours)
            g_i = obj.global_interaction(obj.pos');
            if ~isempty(neighbours)
                q_ij    = obj.pos' - neighbours(:,1:3);       % Relative vector between agent i and j
                nMag    = neighbours(:,4);
                nDir    = q_ij ./ nMag;
                nL_i    = obj.local_interaction(nMag')';
                L_i     = sum(nL_i .* nDir,1)./length(nL_i);   % Average over nr. of agents
                u_d     = L_i + obj.loglo_int(sqrt(L_i(1)^2+L_i(2)^2)) * g_i;
            else
                u_d     = obj.loglo_int(0) * g_i;
            end
            u_d_n   = sqrt(u_d(1)^2 + u_d(2)^2); % + u_d(3)^2);
            if u_d_n > obj.v_max
                u_d = u_d / u_d_n * obj.v_max;
            end
            d_i             = -obj.genome(1)*(u_d - obj.prev_vd);   % Calculate dissipative energy
            v_d             = u_d + d_i;
            obj.prev_vd     = v_d;
            obj.vel_cost    = obj.vel_cost + (g_i(1) - v_d(1))^2 + (g_i(2) - v_d(2))^2;
        end
        
        function y = local_interaction(x)
            y       = x;
        end
        
        function y = loglo_int(obj, L_i_n)
            % Returns a 0-1 scalar of how much of g_i is taken into account
            % dependant on the size of L_i
            y = (max(0,obj.v_max - obj.genome(2) * L_i_n)./obj.v_max).^2;
        end
        
        function X = getAgentFunction(obj,x)
            X = obj.local_interaction(x);
        end
        
        function g = global_interaction(obj,x)
            x = x - obj.c_pos;
            g = obj.fieldFunction(x, obj.field_varargin);
        end
        
        function g_i = pointField(~, pos, inputArgs)
            % pos, v_max
            R               = sqrt(pos(1)^2 + pos(2)^2);
            g_i             = (R>0.5) * inputArgs{1} * [-pos(1) -pos(2) 0]./R;
        end
        
        function g_i = bucketField(obj, pos, inputArgs)
            % pos, v_max, v_min
            pos_n           = sqrt(pos(1)^2+pos(2)^2);
            g_in            = (1 - 1 / (1 + exp(6 / (1.375 * obj.circle_packing_radius) * (pos_n - (1.375 * obj.circle_packing_radius) )))) * inputArgs{1};
            g_id            = [-pos(1) -pos(2) 0]./pos_n;
            g_i             = max(inputArgs{2},min(inputArgs{1},g_in)) * g_id;
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