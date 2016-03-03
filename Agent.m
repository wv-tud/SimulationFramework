classdef Agent < handle
    %AGENT Agent class modelling an agent
    %   Agent class modelling an agent
    
    properties
        % Agent properties
        collision_range     = 0.3;                  % Collision_range [m]
        v_max               = 1;                    % Max v in m/s
        th_max              = 200/360*2*pi();       % Max theta in rad/s
        t_mem               = 20;                   % Memory [timesteps]
        yaw_acc             = 0.95;                 % Accuracy of yaw
        v_acc               = 0.95;                 % Accuracy of v_d implementation
        % Camera properties
        cam_dir             = [0 -10/360*2*pi()];   % Camera direction [radian yaw, radian pitch]
        cam_fov             = 175/360*2*pi();       % Camera FOV [radian]
        cam_range           = 3;                    % Camera range [m]
        cam_acc             = 0.90;                 % Accuracy of Camera   
        % Non-optional properties
        neighbours          = {};                   % Structure to save neighbours
        u_d_decom           = {};                   % Structure to save decomposed v_d into seperate signals
        noise_v             = [];                   % Velocity noise
        noise_th            = [];                   % Yaw noise
        noise_neighbour     = [];                   % Camera noise
        pos                 = [];                   % Matrix containing positions (t,[x y z])
        heading             = [];                   % Matrix containing headings (t,[yaw pitch])
        collisions          = 0;                    % Matrix containing collisions (t,id)
        arena;                                      % Arena object passing property
        id;                                         % UID of agent
    end
    
    methods
        function obj = Agent(arenaC,id,pos,head)
            obj.arena   = arenaC;
            obj.id      = id;
            % Used to initialize the Agent and create matrices
            nT                  = obj.arena.T/obj.arena.dt+1;
            obj.pos             = zeros(nT,3);
            obj.heading         = zeros(nT,2);
            obj.collisions      = zeros(nT,1);
            obj.pos(1,:)        = pos;
            obj.heading(1,:)    = head;
            obj.noise_v         = zeros(nT,3);
            obj.noise_th        = zeros(nT,1);
            obj.noise_neighbour = zeros(nT,obj.arena.nAgents,3);
        end
        
        function obj = Update(obj,neighbours)
            obj.neighbours{obj.arena.t}     = obj.buildNeighbourMatrix(neighbours);                     % Using detected neighbours build the matrix
            v_d_ideal                       = obj.scalableShapeFormation();                             % Determine v_d
            [v_d,v_d_n,theta_change]        = obj.agentDynamics(v_d_ideal);                             % Apply agent dynamics to desired velocity
            v_dhat                          = v_d/v_d_n;                                                % Normalised v_d
            theta                           = atan2(v_dhat(2),v_dhat(1));                               % Yaw angle of v_d
            phi                             = atan2(v_dhat(3),v_d_n);                                   % Pitch angle of v_d
            v_d_prev                        = (obj.pos(obj.arena.t,:)-obj.pos(max(1,obj.arena.t-1),:)); % Calculate dV
            v_noise_range                   = [obj.v_acc (2-obj.v_acc)].*sqrt(sum((v_d-v_d_prev).^2));  % Range of v noise
            v_noise                         = (rand(1,3)-0.5).*(v_noise_range(2)-v_noise_range(1));     % Generate v noise
            obj.noise_v(obj.arena.t,:)      = v_noise;
            th_noise_range                  = [obj.yaw_acc (2-obj.yaw_acc)].*(theta_change);            % Range of yaw noise
            th_noise                        = (rand(1)-0.5).*(th_noise_range(2)-th_noise_range(1));     % Generate yaw noise
            obj.noise_th(obj.arena.t)       = th_noise;
            obj.pos(obj.arena.t+1,:)        = obj.pos(obj.arena.t,:) + v_d + v_noise;                   % Save agent position
            obj.heading(obj.arena.t+1,:)    = [theta+th_noise,phi];                                     % Save agent heading
        end
                
        function m_neighbours = buildNeighbourMatrix(obj,neighbours)
            m_neighbours    = [];
            newStart        = 0;
            if obj.arena.t > 1 && ~isempty(obj.neighbours{obj.arena.t-1})
                m_neighbours = obj.neighbours{obj.arena.t-1}(obj.neighbours{obj.arena.t-1}(:,2) > (obj.arena.t-obj.t_mem),:);                           % Select neighbours from memory which haven't degraded
                m_neighbours = m_neighbours(logical(sum(m_neighbours(:,1)*ones(size(neighbours))==ones(size(m_neighbours(:,1)))*neighbours,2)==0),:);   % Only keep which we cant see
                newStart = size(m_neighbours,1);
%                 for i=1:newStart
%                     q_j     = m_neighbours(i,3:5);                                                  % Select position of agent j
%                     g_j     = feval(obj.g_fun,obj.arena.t*obj.arena.dt,q_j,obj.v_max*obj.arena.dt); % Find global attractor for agent j
%                     g_jn    = sqrt(g_j(1)^2+g_j(2)^2+g_j(3)^2);     % Calculate norm of g_j
%                     if g_jn > obj.v_max*obj.arena.dt                % See if norm is large than max speed
%                         g_j = obj.v_max*obj.arena.dt*g_j/g_jn;      % If so resize to max speed
%                     end
%                     m_neighbours(i,3:5) = q_j + g_j;                % Update memory position with global attractor. ASSUMPTION: Agent j is moving towars centre point just like me
%                 end
            end
            m_neighbours = [m_neighbours; zeros(length(neighbours),5)];
            randN = rand(length(neighbours),3);
            for j = 1:length(neighbours)
                j_pos                                   = obj.arena.agents{neighbours(j)}.pos(obj.arena.t,:);                           % Get position from arena object
                noise_range                             = [obj.cam_acc (2-obj.cam_acc)].*sqrt(sum((obj.pos(obj.arena.t,:)-j_pos).^2));  % Add sensor noise
                noise                                   = (randN(j,:)-0.5).*(noise_range(2)-noise_range(1));                            % Generate noise
                obj.noise_neighbour(obj.arena.t,j,:)    = noise;
                j_pos                                   = j_pos + noise;
                m_neighbours(newStart+j,:)              = [neighbours(j) obj.arena.t j_pos]; % Not yet in memory - insert
            end
        end
        
        function [v_d,v_d_n,theta] = agentDynamics(obj,v_d)
            max_yaw_t   = obj.th_max*obj.arena.dt;  % Maximum yaw in dt
            v_max_t     = obj.v_max*obj.arena.dt;   % Maximum v in dt
            v_d_n       = sqrt(v_d(1)^2+v_d(2)^2+v_d(3)^2);     % norm of desired v
            v_dhat      = v_d/v_d_n;                            % Direction of desired v / normalised
            heading_vec = [1 0 0]*obj.arena.rotMat(-obj.heading(obj.arena.t,1));    % Vector in the direction of the heading 
            angles      = atan2(cross(heading_vec,v_dhat),dot(heading_vec,v_dhat)); % Determine the smallest angle between heading and v_dhat
            theta       = angles(3);                                                % Yaw angle
            
            swing_angle = obj.arena.dt*0.75*(pi()-obj.cam_fov)*cos(2*pi()/2*obj.arena.t*obj.arena.dt + obj.id/obj.arena.nAgents*2*pi())*2*pi()/2;   % Calculate swing angle
            v_d         = v_d*obj.arena.rotMat(swing_angle);                                                                                        % Rotate v_d with swing angle
            
            if abs(theta) > max_yaw_t                                               % See if yaw anlgle > max yaw
                angle_v         = (0.5*obj.cam_fov-max_yaw_t)/2;                    % Find middle between max yaw and edge of FOV
                v_d_n           = v_max_t*(1-1./(1+exp(-14*(abs(theta)-(0.5*obj.cam_fov-angle_v)))));                       % Use smooth step to decrease norm(v_d) as angle gets larger than max yaw, middle of decrease is angle_v, update speed norm
                v_d             = v_d_n * [1 0 0] * obj.arena.rotMat(-obj.heading(obj.arena.t,1)-sign(theta)*max_yaw_t);    % Calculate new v_d using speed limit and yaw limit             
                theta           = (theta/abs(theta))*max_yaw_t;     % Update heading change
            elseif v_d_n > v_max_t
                v_d_n   = v_max_t;          % Update speed norm
                v_d     = v_d_n*v_dhat;     % Update desired velocity
            end
        end
        
        function plotVelocityComponents(obj)
            H = figure(2);
            Fpos = get(H,'pos');
            set(H,'Position',[Fpos(1) Fpos(2) 1000 600]);
            h = uicontrol('style','slider','units','pixel','position',[20 20 960 20],'min',1,'max',obj.arena.nAgents,'sliderstep',[1/(obj.arena.nAgents-1) 1/(obj.arena.nAgents-1)],'Value',obj.id);
            uicontrol('style','text','units','pixel','position',[490 0 54 20],'String',strcat(['Agent ' num2str(obj.id)]));
            addlistener(h,'ContinuousValueChange',@(hObject, event) obj.arena.agents{round(h.Value)}.plotVelocityComponents());

            subplot(4,1,1);
            plot(obj.arena.dt:obj.arena.dt:obj.arena.T,sqrt(sum((obj.u_d_decom.g+obj.u_d_decom.L+obj.u_d_decom.d).^2,2)));
            hold on;
            plot(0:obj.arena.dt:obj.arena.T,sqrt(sum(obj.noise_v.^2,2)));
            plot([obj.arena.dt obj.arena.T],[1 1]*obj.v_max*obj.arena.dt,'k--');
            hold off;
            axis([obj.arena.dt obj.arena.T 0 2.5*obj.v_max*obj.arena.dt]);
            title('v_d - Total');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,2);
            plot(obj.arena.dt:obj.arena.dt:obj.arena.T,sqrt(sum(obj.u_d_decom.g.^2,2)));
            hold on;
            plot([obj.arena.dt obj.arena.T],[1 1]*obj.v_max*obj.arena.dt,'k--');
            hold off;
            axis([obj.arena.dt obj.arena.T 0 2.5*obj.v_max*obj.arena.dt]);
            title('g_i - Global attractor');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,3);
            plot(obj.arena.dt:obj.arena.dt:obj.arena.T,sqrt(sum(obj.u_d_decom.L.^2,2)));
            hold on;
            plot([obj.arena.dt obj.arena.T],[1 1]*obj.v_max*obj.arena.dt,'k--');
            hold off; 
            axis([obj.arena.dt obj.arena.T 0 2.5*obj.v_max*obj.arena.dt]);
            title('L_i - Attr/repul pair');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
            
            subplot(4,1,4);
            plot(obj.arena.dt:obj.arena.dt:obj.arena.T,sqrt(sum(obj.u_d_decom.d.^2,2)));
            hold on;
            plot([obj.arena.dt obj.arena.T],[1 1]*0.1*obj.v_max*obj.arena.dt,'k--');
            hold off;
            axis([obj.arena.dt obj.arena.T 0 0.1*obj.v_max*obj.arena.dt]);
            title('d_i - Dissapative');
            xlabel('Time [s]');
            ylabel('velocity [m/s]');
        end
        
        function F = plotGlobalAttraction(obj,x_arr,y_arr,varargin)
            g           = 0;
            resfac      = 20;
            [x,y]       = meshgrid(x_arr,y_arr);
            x           = reshape(x,[],1);
            y           = reshape(y,[],1);
            if ~isempty(varargin)
                if isa(varargin{1},'function_handle')
                    g = varargin{1};
                elseif isa(varargin{1},'double')
                    if length(varargin{1})==3
                        c_pos   = varargin{1};
                        x       = x - c_pos(1);
                        y       = y - c_pos(2);
                    end
                end
            end
            if g==0
                g = obj.g_fun;
            end
            Rij         = [x y zeros(size(x))];
            g_i         = zeros(length(Rij),3);
            for i=1:length(Rij)
                g_i(i,:) = feval(g,0,Rij(i,:),obj.arena.dt*obj.v_max);
            end
            vNorm   = reshape(sqrt(g_i(:,1).^2+g_i(:,2).^2+g_i(:,3).^2),length(y_arr),length(x_arr));
            u = reshape(g_i(:,1),length(y_arr),length(x_arr))./vNorm;
            v = reshape(g_i(:,2),length(y_arr),length(x_arr))./vNorm;
            w = reshape(g_i(:,3),length(y_arr),length(x_arr))./vNorm;
            F = figure(3);
            hold on;
            surf(x_arr,y_arr,vNorm,'EdgeColor','none','LineStyle','none');
            quiver3(x_arr(1:resfac:end),y_arr(1:resfac:end),vNorm(1:resfac:end,1:resfac:end),u(1:resfac:end,1:resfac:end),v(1:resfac:end,1:resfac:end),w(1:resfac:end,1:resfac:end),0,'Color','k');
            hold off;
            axis equal tight;
            h = colorbar();
            title('Global attractor - g_i(r_{ij})');
            xlabel('x position [m]');
            ylabel('y position [m]');
            zlabel('Velocity [m/s]');
            ylabel(h,'Velocity [m/s]');
        end
    end  
end