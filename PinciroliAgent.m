classdef PinciroliAgent < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Pinciroli properties
        seperation_range    = 1;                    % Ideal seperation between agents
        g_fun               = [];                   % Function handle for global pinciroli attractor  
        g_fun2              = [];                   % Second function handle for conditional functions
        g_cond              = 0;                    % Condition for second function handle
        tL                  = 0;
    end
    
    methods
        function obj = PinciroliAgent(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
%             if ~isa(obj.g_fun,'function_handle')
%                 a           = obj.v_max*obj.dt/(((obj.swarmSize*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
%                 obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
%             end
        end
        
        function v_d = calculate_vd(obj)
            v_d = obj.scalableShapeFormation();
        end
        
        function v_d = scalableShapeFormation(obj)
            sigma   = obj.seperation_range + obj.collision_range;
            e       = obj.genome(1);
            eps     = obj.genome(2);
            q_i     = obj.pos' - obj.c_pos;                 % Vector of current position and c
            g_i     = obj.global_interaction(q_i);
%             if isa(obj.g_fun2,'function_handle')
%                 if norm(q_i(1:2)) >= obj.g_cond
%                     g_i     = feval(obj.g_fun,obj.t*obj.dt,q_i,obj.v_max*obj.dt);  % Gathering
%                 else
%                     g_i     = feval(obj.g_fun2,obj.t*obj.dt,q_i,obj.v_max*obj.dt);  % Gathering2
%                 end
%             else
%                 g_i     = feval(obj.g_fun,obj.t*obj.dt,q_i,obj.v_max*obj.dt);  % Gathering 
%             end
            L_i    = [0 0 0];                                                  % Lattice formation
            d_i    = [0 0 0];                                                  % Dissipative energy
            if ~isempty(obj.neighbours{obj.t})
                %tLn     = 0;
                for j=1:size(obj.neighbours{obj.t},1)
                    q_ij    = q_i-(obj.neighbours{obj.t}(j,3:5) - obj.c_pos);                          % Relative vector between agent i and j
                    q_ijn   = sqrt(q_ij(1)^2+q_ij(2)^2+q_ij(3)^2);                                                  % Normalised relative vector
                    %tLt     = tic;
                    L_i     = L_i + 12*e/q_ijn*((sigma/q_ijn)^12-(sigma/q_ijn)^6)*[q_ij(1)/q_ijn q_ij(2)/q_ijn 0];  % Calculate Lattice formation
                    %tLn     = tLn + toc(tLt);
                end
                %obj.tL = (obj.tL * (obj.t-1) + toc(tLt)/length(obj.neighbours{obj.t})) / obj.t;
                %fprintf('%0.10f\n', obj.tL);
                L_i = L_i / length(obj.neighbours{obj.t});     % Average over nr. of agents
            end
            if obj.t > 1
                d_i = -eps*(L_i + g_i - (obj.u_d_decom.g(obj.t-1,:)+obj.u_d_decom.L(obj.t-1,:)+obj.u_d_decom.d(obj.t-1,:)));   % Calculate dissipative energy
            end
            u_d = g_i + L_i + d_i;                  % Sum to find u_d
            obj.u_d_decom.g(obj.t,:) = g_i;   % Save to array for plotting
            obj.u_d_decom.L(obj.t,:) = L_i;   % Save to array for plotting
            obj.u_d_decom.d(obj.t,:) = d_i;   % Save to array for plotting
            v_d = u_d;                              % Convert u_d to v_d
        end
        
        function y = local_interaction(obj,x)
            sigma   = obj.seperation_range + obj.collision_range;
            y       = 12*obj.genome(1)/x*((sigma/x)^12-(sigma/x)^6);
        end
    end
    
end

