classdef Mission < Arena
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mission_type    = 'default';
        waypointArr     = [];
        n_ellipse       = 1;
        circle_radius   = 2;
        separation_range = 1.75;
    end
    
    methods
        function obj = Mission(varargin)
            % Initialise mission and set mission type
            obj = obj@Arena(varargin{1});
            if nargin > 1
                obj.mission_type = varargin{2};
                if strcmp(varargin{2},'waypoint') && nargin == 3
                    obj.waypointArr = varargin{3};
                elseif strcmp(varargin{2},'ellipseTracking') && nargin == 3
                    obj.n_ellipse = varargin{3};
                end
            end
        end
        
        function Simulate(obj,varargin)
            obj.initMission();
            obj.Simulate@Arena(varargin);
        end
        
        function initMission(obj,varargin)
            switch(obj.mission_type)
                case 'cyberzooCW'
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);      % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
%                     obj.agent_conf  = obj.mergeStruct(obj.agent_conf, struct(   'g_fun', @(varargin) [...
%                         cosd(90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(1))-sind(90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(2)) ...
%                         sind(90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(1))+cosd(90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(2)) ...
%                         0], 'g_fun2', @(varargin)[...
%                         cosd(90 + 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(1))-sind(90 + 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(2)) ...
%                         sind(90 + 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(1))+cosd(90 + 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(2)) ...
%                         0], 'g_cond', obj.circle_radius, 'seperation_range', obj.separation_range));
                   clear tmp_agent;
                case 'cyberzooCCW'
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);      % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
%                     obj.agent_conf  = obj.mergeStruct(obj.agent_conf, struct(   'g_fun', @(varargin) [...
%                         cosd(-90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(1))-sind(-90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(2)) ...
%                         sind(-90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(1))+cosd(-90*obj.agent_conf.g_cond/norm(-varargin{2}(1:2)))*varargin{3}*(-varargin{2}(2)) ...
%                         0], 'g_fun2', @(varargin) [...
%                         cosd(-90 - 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(1))-sind(-90 - 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(2)) ...
%                         sind(-90 - 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(1))+cosd(-90 - 90 * (obj.agent_conf.g_cond - norm(-varargin{2}(1:2)))/obj.agent_conf.g_cond)*varargin{3}*(-varargin{2}(2)) ...
%                         0], 'g_cond', obj.circle_radius, 'seperation_range', obj.separation_range));
                    clear tmp_agent;
                case 'cyberzooBucket'
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);      % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
%                     obj.agent_conf  = obj.mergeStruct(obj.agent_conf, struct(   'g_fun', @(varargin) (1 - 1 / (1 + exp(4 / 1.75 * (norm(-varargin{2}(1:2)) - 1.75)))) * varargin{3} / norm(-varargin{2}(1:2)) * [-varargin{2}(1) -varargin{2}(2) 0], 'seperation_range', obj.separation_range));
                    clear tmp_agent;
                case 'lineTracking'
                    %obj.p_axe_lim   = [-52.5 52.5 -34 34];                  % Set field to football field
                    %obj.p_mov_axe   = 0;                                    % No moving axes
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);  % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
                    v_line          = 0.75*tmp_agent.v_max;                 % Set speed of point to track
                    obj.c_fun       = @(t) [...                             % Define function for line tracking
                        ((t<50/v_line).*(v_line*t) + (t>50/v_line).*(t<150/v_line).*(50-v_line*(t-50/v_line)) + (t>150/v_line).*(t<200/v_line).*(-50+v_line*(t-150/v_line))) ...
                        ((t>200/v_line).*(t<230/v_line).*(v_line*t) + (t>230/v_line).*(t<260/v_line).*(30-v_line*(t-230/v_line)) + (t>260/v_line).*(t<290/v_line).*(-30+v_line*(t-260/v_line))) ...
                        10];
                    obj.T           = ceil(290/v_line/10)*10;               % Rount time to completion time closest 10s
                    %obj.agent_conf  = struct('g_fun',tmp_agent.g_fun);      % Use default g_fun
                    clear tmp_agent;                                        % Put tmp agent out with the trash
                case 'ellipseTracking'
                    %obj.p_axe_lim   = [-52.5 52.5 -34 34];                      % Set field to football field
                    %obj.p_mov_axe   = 0;                                        % No moving axes
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);      % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
                    v_circ          = 0.75*tmp_agent.v_max;                     % Set speed of point to track
                    a               = 45;                                       % Size of ellipse
                    b               = 25;                                       % Size of ellipse
                    T_des           = sqrt((a*2*pi())^2+(b*2*pi())^2)/v_circ;   % Calculate time required to not exceed v_circ
                    T_des           = obj.n_ellipse*T_des;
                    obj.c_fun       = @(t) [a*sin(2*pi()/T_des*(t)) b*cos(2*pi()/T_des*(t)) 10];
                    obj.T           = ceil(T_des/10)*10;                        % Rount time to completion time closest 10s
                    %obj.agent_conf  = struct('g_fun',tmp_agent.g_fun);          % Use default g_fun
                    clear tmp_agent;                                            % Put tmp agent out with the trash
                case 'swirl'
                    tmp_agent       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);      % Create tmp agent to read properties
                    tmp_agent       = obj.mergeStruct(tmp_agent,obj.agent_conf);
                    a               = tmp_agent.v_max*obj.dt/(((obj.nAgents*(tmp_agent.collision_range+tmp_agent.seperation_range)^2*sqrt(3)/2)/pi()));
                    %obj.agent_conf  = struct('g_fun',@(varargin)  min(a*(norm(varargin{2})).^2,varargin{3}).*varargin{2}./norm(varargin{2}).*[-10 -10 0]*[cos((1/(1+0.02*norm(varargin{2})))*0.5*pi()) -sin((1/(1+0.02*norm(varargin{2})))*0.5*pi()) 0; sin((1/(1+0.02*norm(varargin{2})))*0.5*pi()) cos((1/(1+0.02*norm(varargin{2})))*0.5*pi()) 0; 0 0 1]);
                    clear tmp_agent;
                case 'waypoint'
                    if ~isempty(obj.waypointArr)
                        tmp_agent                       = Agent_pinciroli(obj,0,[0 0 0],[0 0]);  % Create tmp agent to read properties
                        tmp_agent                       = obj.mergeStruct(tmp_agent,obj.agent_conf);
                        v_wp                            = 0.75*tmp_agent.v_max;                 % Set speed of waypoints if no pre-set time is given
                        clear tmp_agent;                                                        % Put tmp agent out with the trash
                        e_str                           = '0';                                  % String containing zero, allows for function stacking in for loop
                        t_wp                            = zeros(length(obj.waypointArr),1);     % Pre-allocate
                        wayPoints                       = zeros(length(obj.waypointArr),3);     % Pre-allocate
                        wayPoints(1,:)                  = obj.waypointArr(1,1:3);
                        for i=2:size(obj.waypointArr,1)
                            wayPoints(i,:)  = obj.waypointArr(i,1:3);                           % Get current waypoint
                            if length(obj.waypointArr(i,:))==4                                  % See if waypoint time is defined
                                t_wp(i)     = obj.waypointArr(i,4);                             % Get waypoint time
                            else
                                wp_dist     = norm(wayPoints(i,1:2)-wayPoints(i-1,1:2));        % Calculate distance to waypoint from prev waypoint
                                t_wp(i)     = wp_dist/v_wp;                                     % Calculate waypoint time based on 75% v_max
                            end
                            e_str = strcat([e_str ' + (t>' num2str(sum(t_wp(1:i-1))) ').*(t<' num2str(sum(t_wp(1:i))) ').*((t-' num2str(sum(t_wp(1:i-1))) ')/' num2str(t_wp(i)) '.*([' num2str(wayPoints(i,:)) '] - [' num2str(wayPoints(i-1,:)) '])+[' num2str(wayPoints(i-1,:)) '])']); % Create c_function to lead from prev to current waypoint in t_wp seconds
                        end
                        eval(strcat(['obj.c_fun = @(t) ' e_str ';']));
                        obj.T = ceil(sum(t_wp)*10)/10;
                    end
                otherwise
                    % Keep all settings as-is
            end
        end
    end
    
end

