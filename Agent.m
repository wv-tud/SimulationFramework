classdef Agent < handle
    %AGENT Agent class modelling an agent
    %   Agent class modelling an agent
    
    properties
        % Agent properties
        collision_range     = 0.3;                  % Collision_range [m]
        v_max               = 1;                    % Max v in m/s
        th_max              = 200/180*pi();       % Max theta in rad/s
        t_mem               = 20;                   % Memory [timesteps]
        yaw_acc             = 0.95;                 % Accuracy of yaw
        v_acc               = 0.95;                 % Accuracy of v_d implementation
        % Camera properties
        cam_dir             = [0 -10/180*pi()];   % Camera direction [radian yaw, radian pitch]
        cam_fov             = 152/180*pi();       % Camera FOV [radian]
        cam_range           = 4;                    % Camera range [m]
        cam_acc             = 0.85;                 % Accuracy of Camera   
        % Non-optional properties
        neighbours          = {};                   % Structure to save neighbours
        u_d_decom           = {};                   % Structure to save decomposed v_d into seperate signals
        pos                 = [];                   % Matrix containing positions (t,[x y z])
        heading             = [];                   % Matrix containing headings (t,[yaw pitch])
        vel                 = [];
        genome              = [];
        collisions          = 0;                    % Matrix containing collisions (t,id)
        net                 = @(x) 0;               % Placeholder for simpleNN network
        vel_cost            = 0;
        dist_cost           = 0;
        id;                                         % UID of agent
        dt                  = 0;
        t                   = 0;
        T                   = 0;
        swarmMode;
        swarmSize;
        c_pos;
        circle_packing_radius;
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
            nT                  = obj.T/obj.dt+1;
            obj.pos             = pos;
            obj.vel             = [0 0 0];
            obj.heading         = head;
            obj.collisions      = zeros(nT,1);
        end
        
        function [v_d,theta,phi] = Update(obj,t,c_pos,pos,heading,vel,neighbours, agent_positions)
            obj.t           = t;
            obj.pos         = pos;
            obj.c_pos       = c_pos;
            obj.vel         = vel;
            obj.heading     = heading;
            obj.neighbours{obj.t}   = obj.buildNeighbourMatrix(neighbours, agent_positions);                     % Using detected neighbours build the matrix
            v_d                     = obj.calculate_vd();
            v_dhat                  = v_d/norm(v_d);
            if obj.swarmMode == 1
                phi                           = atan2(v_dhat(2),v_dhat(1));                           % Yaw angle of v_d
                theta                         = atan2(v_dhat(3),v_d_n);                               % Pitch angle of v_d
            else
                phi                           = atan2(obj.u_d_decom.g(obj.t,2),obj.u_d_decom.g(obj.t,1));                           % Yaw angle of v_d
                theta                         = atan2(obj.u_d_decom.g(obj.t,3),0); 
            end
            if ~isempty(obj.neighbours{obj.t})
                nearest_neighbours = sort(abs((obj.seperation_range + obj.collision_range)./sqrt(obj.neighbours{obj.t}(:,3).^2 + obj.neighbours{obj.t}(:,4).^2) - 1));
                obj.dist_cost      = obj.dist_cost + mean(nearest_neighbours(1:min(length(obj.neighbours{obj.t}(:,1)),3)).^2);
            end
            obj.vel_cost                      = obj.vel_cost + norm(v_d - obj.u_d_decom.g(obj.t,:)).^2;                             % Apply agent dynamics to desired velocity          
            %pause;
%             v_d_prev                        = (obj.pos(obj.t,:)-obj.pos(max(1,obj.t-1),:)); % Calculate dV
%             v_noise_range                   = [obj.v_acc (2-obj.v_acc)].*sqrt(sum((v_d-v_d_prev).^2));  % Range of v noise
%             v_noise                         = (rand(1,3)-0.5).*(v_noise_range(2)-v_noise_range(1));     % Generate v noise
%             obj.noise_v(obj.t,:)      = v_noise;
%             th_noise_range                  = [obj.yaw_acc (2-obj.yaw_acc)].*(theta_change);            % Range of yaw noise
%             th_noise                        = (rand(1)-0.5).*(th_noise_range(2)-th_noise_range(1));     % Generate yaw noise
%             obj.noise_th(obj.t)       = th_noise;
%             obj.pos(obj.t+1,:)        = obj.pos(obj.t,:) + v_d + v_noise;                   % Save agent position
%             obj.heading(obj.t+1,:)    = [theta+th_noise,phi];                                     % Save agent heading
        end
                
        function m_neighbours = buildNeighbourMatrix(obj,neighbours,agent_positions)
            m_neighbours    = [];
            newStart        = 0;
            if obj.t > 1 && ~isempty(obj.neighbours{obj.t-1})
                m_neighbours = obj.neighbours{obj.t-1}(obj.neighbours{obj.t-1}(:,2) > (obj.t-obj.t_mem),:);                           % Select neighbours from memory which haven't degraded
                m_neighbours = m_neighbours(logical(sum(m_neighbours(:,1)*ones(size(neighbours))==ones(size(m_neighbours(:,1)))*neighbours,2)==0),:);   % Only keep which we cant see
                newStart = size(m_neighbours,1);
            end
            m_neighbours = [m_neighbours; zeros(length(neighbours),5)];
            randN = rand(length(neighbours),3);
            for j = 1:length(neighbours)
                j_pos                                   = agent_positions(neighbours(j),:)';
                noise_range                             = [obj.cam_acc (2-obj.cam_acc)].*sqrt(sum((obj.pos-j_pos).^2));  % Add sensor noise
                noise                                   = (randN(j,:)-0.5).*(noise_range(2)-noise_range(1));                            % Generate noise
                j_pos                                   = j_pos + noise';
                m_neighbours(newStart+j,:)              = [neighbours(j) obj.t j_pos']; % Not yet in memory - insert
            end
        end
        
        function plotVelocityComponents(obj)
            H = figure(2);
            Fpos = get(H,'pos');
            set(H,'Position',[Fpos(1) Fpos(2) 1000 600]);
            h = uicontrol('style','slider','units','pixel','position',[20 20 960 20],'min',1,'max',obj.swarmSize,'sliderstep',[1/(obj.swarmSize-1) 1/(obj.swarmSize-1)],'Value',obj.id);
            uicontrol('style','text','units','pixel','position',[490 0 54 20],'String',strcat(['Agent ' num2str(obj.id)]));
            %addlistener(h,'ContinuousValueChange',@(hObject, event) obj.arena.agents{round(h.Value)}.plotVelocityComponents());

            subplot(4,1,1);
            plot(obj.dt:obj.dt:obj.T,sqrt(sum((obj.u_d_decom.g+obj.u_d_decom.L+obj.u_d_decom.d).^2,2)));
            hold on;
%             plot(0:obj.dt:obj.T,sqrt(sum(obj.noise_v.^2,2)));
            plot([obj.dt obj.T],[1 1]*obj.v_max*obj.dt,'k--');
            hold off;
            axis([obj.dt obj.T 0 2.5*obj.v_max*obj.dt]);
            title('v_d - Total');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,2);
            plot(obj.dt:obj.dt:obj.T,sqrt(sum(obj.u_d_decom.g.^2,2)));
            hold on;
            plot([obj.dt obj.T],[1 1]*obj.v_max*obj.dt,'k--');
            hold off;
            axis([obj.dt obj.T 0 2.5*obj.v_max*obj.dt]);
            title('g_i - Global attractor');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,3);
            plot(obj.dt:obj.dt:obj.T,sqrt(sum(obj.u_d_decom.L.^2,2)));
            hold on;
            plot([obj.dt obj.T],[1 1]*obj.v_max*obj.dt,'k--');
            hold off; 
            axis([obj.dt obj.T 0 2.5*obj.v_max*obj.dt]);
            title('L_i - Attr/repul pair');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,4);
            plot(obj.dt:obj.dt:obj.T,sqrt(sum(obj.u_d_decom.d.^2,2)));
            hold on;
            plot([obj.dt obj.T],[1 1]*0.1*obj.v_max*obj.dt,'k--');
            hold off;
            axis([obj.dt obj.T 0 0.1*obj.v_max*obj.dt]);
            title('d_i - Dissapative');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
        end
        
        function F = plotGlobalAttraction(obj,x_arr,y_arr,varargin)
            resfac      = 20;
            [x,y]       = meshgrid(x_arr,y_arr);
            x           = reshape(x,[],1);
            y           = reshape(y,[],1);
            if ~isempty(varargin)
                if isa(varargin{1},'function_handle')
                elseif isa(varargin{1},'double')
                    if length(varargin{1})==3
                        cc_pos   = varargin{1};
                        x       = x - cc_pos(1);
                        y       = y - cc_pos(2);
                    end
                end
            end
            Rij         = [x y zeros(size(x))];
            g_i         = zeros(length(Rij),3);
            for i=1:length(Rij)
                g_i(i,:) = obj.global_interaction(Rij(i,:));
                %g_i(i,:) = feval(g,0,Rij(i,:),obj.dt*obj.v_max);
            end
            vNorm   = reshape(sqrt(g_i(:,1).^2+g_i(:,2).^2+g_i(:,3).^2),length(y_arr),length(x_arr));
            u = reshape(g_i(:,1),length(y_arr),length(x_arr))./vNorm;
            v = reshape(g_i(:,2),length(y_arr),length(x_arr))./vNorm;
            w = reshape(g_i(:,3),length(y_arr),length(x_arr))./vNorm;
            F = figure(3);
            hold on;
            surf(x_arr,y_arr, vNorm,'EdgeColor','none','LineStyle','none');
            quiver3(x_arr(1:resfac:end),y_arr(1:resfac:end), vNorm(1:resfac:end,1:resfac:end),u(1:resfac:end,1:resfac:end),v(1:resfac:end,1:resfac:end),w(1:resfac:end,1:resfac:end),0,'Color','k');
            hold off;
            axis equal tight;
            h = colorbar();
            title('Global attractor - g_i(r_{ij})');
            xlabel('x position [m]');
            ylabel('y position [m]');
            zlabel('Velocity [m/s]');
            ylabel(h,'Velocity [m/s]');
        end
        
        function X = getAgentFunction(obj,x)
            X = zeros(size(x));
            for r=1:length(x)
                X(r) = obj.local_interaction(x(r)); 
            end
        end
        
        function y = local_interaction(x)
            y=0;
        end
        
        function g = global_interaction(obj,x)
            g = obj.globalField(x, obj.seperation_range + obj.collision_range, obj.v_max);
        end
        
        function g_i = globalField(obj,pos,seperation_distance,v_max)
            bucket_radius   = obj.circle_packing_radius(obj.swarmSize) * seperation_distance;
            g_i             = (1.0 - 1 / (1 + exp(5 / bucket_radius * (norm(pos(1:2)) - bucket_radius)))) * v_max / norm(pos(1:2)) * [-pos(1) -pos(2) 0];
        end
    end  
end