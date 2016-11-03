classdef simpleNNAgent < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % NeuralAgent properties
        seperation_range    = 1;                    % Ideal seperation between agents
        g_fun               = [];                   % Function handle for global pinciroli attractor  
        g_fun2              = [];                   % Second function handle for conditional functions
        g_cond              = 0;                    % Condition for second function handle
        tL                  = 0;
    end
    
    methods
        function obj = simpleNNAgent(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
            if ~isa(obj.g_fun,'function_handle')
                a           = obj.v_max*obj.arena.dt/(((obj.arena.nAgents*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
                obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
            end
        end
        
        function y = nnNormI(obj,x)
            y = (x - (obj.collision_range + obj.seperation_range))./(obj.cam_range);
        end
        
        function v_d = calculate_vd(obj)
            v_d = obj.evaluateNN();
        end
        
        function v_d = evaluateNN(obj)
            eps     = 0.1;
            q_i     = obj.pos(obj.arena.t,:) - obj.arena.c_pos(obj.arena.t,:);                 % Vector of current position and c
            sigma   = obj.collision_range + obj.seperation_range;
            if isa(obj.g_fun2,'function_handle')
                if norm(q_i(1:2)) >= obj.g_cond
                    g_i     = feval(obj.g_fun,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering
                else
                    g_i     = feval(obj.g_fun2,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering2
                end
            else
                g_i     = feval(obj.g_fun,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering 
            end
            L_i     = [0 0 0];                                                  % Lattice formation
            d_i     = [0 0 0];                                                  % Dissipative energy
            if ~isempty(obj.neighbours{obj.arena.t})
                %tLn = 0;
                vL_i    = zeros(size(obj.neighbours{obj.arena.t},1),3);
                vq_ijn  = zeros(size(obj.neighbours{obj.arena.t},1),1);
                for j=1:size(obj.neighbours{obj.arena.t},1)
                    q_ij    = q_i - (obj.neighbours{obj.arena.t}(j,3:5) - obj.arena.c_pos(obj.arena.t,:));                          % Relative vector between agent i and j
                    q_ijn   = sqrt(q_ij(1)^2+q_ij(2)^2+q_ij(3)^2);                                                  % Normalised relative vector
                    if q_ijn > 0
                        %tLt = tic;
                        vq_ijn(j) = q_ijn;
                        vL_i(j,:) = [q_ij(1)/q_ijn q_ij(2)/q_ijn 0]; % Calculate Lattice formation  
                        %tLn     = tLn + toc(tLt);
                    end
                    obj.dist_cost = obj.dist_cost + abs(sigma./q_ijn - 1) ./ size(obj.neighbours{obj.arena.t},1);
                end
                L_i = sum(obj.v_max .* repmat(obj.arena.net(obj.nnNormI(vq_ijn)'),3,1)' .* vL_i,1);
                %obj.tL = (obj.tL * (obj.arena.t-1) + toc(tLt)/length(obj.neighbours{obj.arena.t})) / obj.arena.t;
                %fprintf('%0.10f\n', obj.tL);
                L_i = L_i / length(obj.neighbours{obj.arena.t});     % Average over nr. of agents
            end
            if obj.arena.t > 1
                d_i = -eps*(L_i+g_i - (obj.u_d_decom.g(obj.arena.t-1,:)+obj.u_d_decom.L(obj.arena.t-1,:)+obj.u_d_decom.d(obj.arena.t-1,:)));   % Calculate dissipative energy
            end
            u_d = g_i + L_i + d_i;                  % Sum to find u_d
            obj.u_d_decom.g(obj.arena.t,:) = g_i;   % Save to array for plotting
            obj.u_d_decom.L(obj.arena.t,:) = L_i;   % Save to array for plotting
            obj.u_d_decom.d(obj.arena.t,:) = d_i;   % Save to array for plotting
            v_d = u_d;                              % Convert u_d to v_d
        end
        
        function y = local_interaction(obj,x)
            y = 0;
        end
    end
    
end
