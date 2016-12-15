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
        
        function [v_d,theta,phi,g_d] = Update(obj,t,pos,heading,vel,neighbours,agent_positions,agent_distances)
            obj.t                   = t;
            if any(isnan(pos))
                fprintf('Update: pos(%0.f,%0.f,%0.f) t(%0.f)\n',pos(1),pos(2),pos(3),t);
                pause;
            end
            obj.pos                 = pos;
            if isa(obj.c_fun,'function_handle')
                obj.c_pos               = feval(obj.c_fun, t * obj.dt);
            end
            obj.vel                 = vel;
            obj.heading             = heading;
            obj.prev_neighbours     = obj.neighbours;
            obj.neighbours          = obj.buildNeighbourMatrix(neighbours, agent_positions, agent_distances');                     % Using detected neighbours build the matrix
            [v_d,g_d]               = obj.calculate_vd(obj.neighbours(:,3:6));
            if obj.swarmMode == 1
                v_d_n                   = sqrt(v_d(1)^2+v_d(2)^2);
                if v_d_n > 0
                    v_dhat                  = v_d/v_d_n;
                    phi                     = atan2(v_dhat(2),v_dhat(1));                           % Yaw angle of v_d
                    theta                   = 0;
                    %theta                   = atan2(v_dhat(3),v_d_n);                               % Pitch angle of v_d
                else
                    phi                     = 0;
                    theta                   = 0;
                end
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
                pos_noise       = dist_noise .* (obj.pos' - agent_positions(neighbours,:)) ./ max(0.01,agent_distances);
                j_pos           = agent_positions(neighbours,:) - pos_noise;
                m_neighbours(newStart+1:end,:)  = [neighbours' obj.t*ones(nrNeighbours,1) j_pos agent_distances+dist_noise];
            end
        end
        
        function [v_d,g_i] = calculate_vd(obj, neighbours)
            g_i = obj.global_interaction(obj.pos');
            if ~isempty(neighbours)
                q_ij    = obj.pos' - neighbours(:,1:3);       % Relative vector between agent i and j
                nMag    = neighbours(:,4);
                nDir    = q_ij ./ max(0.01, nMag);
                nL_i    = obj.local_interaction(nMag')';
                L_i     = sum(nL_i .* nDir,1)./length(nL_i);   % Average over nr. of agents
                u_d     = L_i + obj.loglo_int(nMag,nDir,g_i) * g_i;
            else
                L_i     = [0 0 0];
                u_d     = g_i;
            end
            u_d_n   = sqrt(u_d(1)^2 + u_d(2)^2); % + u_d(3)^2);
            if u_d_n > obj.v_max
                u_d = u_d / u_d_n * obj.v_max;
            end
            d_i             = -obj.genome(1)*(u_d - obj.prev_vd);   % Calculate dissipative energy
            v_d             = u_d + d_i;
            obj.prev_vd     = v_d;
        end
        
        function y = local_interaction(x)
            y       = x;
        end
        
        function y = loglo_int(obj, nMag, nDir, g_i)
            % Returns a 0-1 scalar of how much of g_i is taken into account
            % dependant on the size of L_i
            %y=1;
            rel_dist        = min(1,(obj.cam_range - nMag)/(obj.cam_range - obj.seperation_range));
            w_neighbours    = norm(g_i) * sum(rel_dist.^2 .* nDir,1)/length(nMag);
            % Find component along g_i
            cg_i            = dot(w_neighbours,g_i)/dot(g_i,g_i)*g_i;
            y               = min(1,max(0, (1.0 - (norm(g_i - cg_i)-norm(g_i))/norm(g_i))))^(2*obj.genome(2));   
%             figure(5);
%             cla
%             hold all;
%             quiver(0,0,g_i(1),g_i(2),'--','DisplayName','g_i')
%             quiver(zeros(length(nMag),1),zeros(length(nMag),1),nMag .* nDir(:,1),nMag .* nDir(:,2),'--')
%             quiver(0,0,w_neighbours(1), w_neighbours(2),'--','DisplayName','w_neighbours')
%             quiver(0,0,cg_i(1), cg_i(2),'--','DisplayName','cg_i')
%             quiver(0,0,y*g_i(1),y*g_i(2),'DisplayName','final g_i');
%             viscircles([0 0],obj.cam_range,'LineWidth',1,'LineStyle','--','Color','black');
%             legend('show');
%             axis equal;
%             set(gca,'XMinorGrid','on')
%             set(gca,'YMinorGrid','on')
%             pause
%             close 5;
            return;
            y = (max(0,obj.v_max - obj.genome(2) * sqrt(mean((obj.cam_range-nL).^2)))./obj.v_max).^2;
            return;
             nrNeighbours = length(nL);
%             y = (1- sum(max(0,(obj.cam_range - nL)).^(nrNeighbours*obj.genome(2)))/(nrNeighbours*obj.cam_range^2));
%             y = max(0,min(1,y));
            if nrNeighbours > 1
                nDir = -nDir;
                n_angles                = obj.smallAngle(atan2(nDir(:,2),nDir(:,1)) - obj.heading(1));
                n_angles(n_angles<0)    = n_angles(n_angles<0) + 2 * pi();
                nInd                    = n_angles < pi();
                n_angles                = n_angles(nInd);
                nL                      = nL(nInd);
                nrNeighbours            = length(n_angles);
            end
            switch nrNeighbours
                case 0
                    y = 1;
                case 1
                    totA    = 1/2 * pi() * (obj.cam_range)^2;
                    theta   = 2 * acos(((obj.cam_range) - (obj.cam_range-nL/2))/(obj.cam_range));
                    if ~isreal(theta)
                        theta = 0;
                    end
                    dA      = (obj.cam_range)^2/2*(theta - sin(theta));
                    y       = max(0,min(1,(totA - dA)/totA));
                otherwise
                    [n_sAngles,sI]  = sort(n_angles);
                    n_sDist         = nL(sI);
                    tri_angles      = diff(n_sAngles);
                    tri_angles      = [n_sAngles(1) tri_angles' pi()-n_sAngles(end)];
                    dA              = 0;
                    if tri_angles(1) > 1/2 * pi()
                        tri_angles(1)   = tri_angles(1) - 1/2 * pi();
                        dA              = dA + 1/2 * 1/2 * pi() * (obj.cam_range)^2;
                    end
                    dA  = dA + 1/2 * (n_sDist(1)/2)^2 * tan(tri_angles(1));
                    for i=2:length(n_angles)
                        dA = dA + 1/2 * (n_sDist(i-1)/2)^2 * tan(tri_angles(i)/2);
                        dA = dA + 1/2 * (n_sDist(i)/2)^2 * tan(tri_angles(i)/2);
                    end
                    if tri_angles(end) > 1/2 * pi()
                        tri_angles(end) = tri_angles(end) - 1/2 * pi();
                        dA              = dA + 1/2 * 1/2 * pi() * (obj.cam_range)^2;
                    end
                    dA      = dA + 1/2 * (n_sDist(end)/2)^2 * tan(tri_angles(end));
                    totA    = 1/4 * pi() * (obj.cam_range)^2;
                    %totA    = nrNeighbours /4 * obj.cam_range^2 * tan(pi()/(2* nrNeighbours));
                    y       = max(0,min(1,dA / totA));
            end
           y = y^obj.genome(2);
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
            g_i             = (R>0.5) * inputArgs{1} * [-pos(1) -pos(2) 0]./max(0.5,R);
        end
        
        function g_i = bucketField(obj, pos, inputArgs)
            % pos, v_max, v_min
            pos_n           = max(0.01, sqrt(pos(1)^2+pos(2)^2));
            if pos_n > 0
                g_in            = (1 - 1 / (1 + exp(6 / (1.375 * obj.circle_packing_radius) * (pos_n - (1.375 * obj.circle_packing_radius) )))) * inputArgs{1};
                g_id            = [-pos(1) -pos(2) 0]./pos_n;
                g_i             = max(inputArgs{2},min(inputArgs{1},g_in)) * g_id;
            else
                g_i             = [0 0 0];
            end
        end
        
        function g_i = circleField(~, pos, inputArgs)
            % pos, radius, v_max, direction, band_width_gain, spiral_gain
            pos_n           = max(0.01, sqrt(pos(1)^2+pos(2)^2));
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
        
        function varargout = plotGlobalAttraction(obj,x_arr,y_arr,varargin)
            resfac      = 20;
            [x,y]       = meshgrid(x_arr,y_arr);
            x           = reshape(x,[],1);
            y           = reshape(y,[],1);
            figure_bool = false;
            noplot      = false;
            if ~isempty(varargin)
                for i=1:length(varargin)
                    if isa(varargin{i},'function_handle')
                        
                    elseif isa(varargin{i},'double')
                        if length(varargin{i})==3
                            obj.c_pos  = varargin{i};
                        end
                    elseif isa(varargin{i},'logical')
                        if ~figure_bool
                            if varargin{i}
                                F       = figure();
                                figure_bool = true;
                            else
                                F       = false;
                                figure_bool = true;
                            end
                        else
                            if varargin{i}
                                noplot = true;
                            else
                                noplot = false;
                            end
                        end
                    end
                end  
            end
            Rij         = [x y zeros(size(x))];
            g_i         = zeros(length(Rij),3);
            for i=1:length(Rij)
                g_i(i,:)    = obj.global_interaction(Rij(i,:));
            end
            vNorm   = max(0.01, reshape(sqrt(g_i(:,1).^2+g_i(:,2).^2+g_i(:,3).^2),length(y_arr),length(x_arr)));
            u       = reshape(g_i(:,1),length(y_arr),length(x_arr))./vNorm;
            v       = reshape(g_i(:,2),length(y_arr),length(x_arr))./vNorm;
            w       = reshape(g_i(:,3),length(y_arr),length(x_arr))./vNorm;
            if ~noplot
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
            if nargout == 1
                varargout{1} = F;
            elseif nargout >= 3
                varargout{1} = x_arr;
                varargout{2} = y_arr;
                varargout{3} = vNorm;
                if nargout == 6
                    varargout{4} = u;
                    varargout{5} = v;
                    varargout{6} = w;
                end
            end
        end
        
        function small_angles = smallAngle(~,angles)
            small_angles        = angles  + floor(abs(angles./(2*pi()))).*(-2.*sign(angles))*pi();
            ipPi                = find(small_angles>pi());
            imPi                = find(small_angles<-pi());
            small_angles(ipPi)  = -2*pi() + small_angles(ipPi);
            small_angles(imPi)  = 2*pi() + small_angles(imPi);
        end
    end
end