classdef neuralnetAgent < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % NeuralAgent properties
        seperation_range    = 1;                    % Ideal seperation between agents
        g_fun               = [];                   % Function handle for global pinciroli attractor  
        g_fun2              = [];                   % Second function handle for conditional functions
        g_cond              = 0;                    % Condition for second function handle
    end
    
    methods
        function obj = neuralnetAgent(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
            if ~isa(obj.g_fun,'function_handle')
                a           = obj.v_max*obj.arena.dt/(((obj.arena.nAgents*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
                obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
            end
        end
        
        function v_d = calculate_vd(obj)
            v_d = obj.evaluateNN();
        end
        
        function v_d = evaluateNN(obj)
            MAX_NEIGHBOURS = 5;
            eps     = 0.1;
            q_i     = obj.pos(obj.arena.t,:) - obj.arena.c_pos(obj.arena.t,:);                 % Vector of current position and c
            if isa(obj.g_fun2,'function_handle')
                if norm(q_i(1:2)) >= obj.g_cond
                    g_i     = feval(obj.g_fun,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering
                else
                    g_i     = feval(obj.g_fun2,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering2
                end
            else
                g_i     = feval(obj.g_fun,obj.arena.t*obj.arena.dt,q_i,obj.v_max*obj.arena.dt);  % Gathering 
            end
            % Lattice formation
            neighbours = zeros(size(obj.neighbours{obj.arena.t},1),4);
            for n=1:size(obj.neighbours{obj.arena.t},1)
                neighbours(n,3:4)   = obj.neighbours{obj.arena.t}(n,3:4) - obj.pos(obj.arena.t,1:2);
                neighbours(n,1)     = max(0,min(1,obj.collision_range / norm(neighbours(n,3:4))));
                heading_vec         = [1 0 0]*obj.arena.rotMat(-obj.heading(obj.arena.t,1));    % Vector in the direction of the current agent heading
                relAngle            = atan2(cross(heading_vec,[neighbours(n,3:4) 0]/norm([neighbours(n,3:4) 0])),dot(heading_vec,[neighbours(n,3:4) 0]/norm([neighbours(n,3:4) 0])));
                neighbours(n,2)     = max(0,min(1,(relAngle(3) + pi())/(2*pi())));
            end
            neighbours = sort(neighbours,1,'descend');
            if size(neighbours,1)>MAX_NEIGHBOURS
                neighbours = neighbours(1:MAX_NEIGHBOURS,:);
            end
            % Radius + angle (normalized)
            input = zeros(1,10);
            input(1:(2*size(neighbours,1))) =  reshape(neighbours(:,1:2)',1,numel(neighbours(:,1:2)));
            % Radius + angle (normalized)
            output = evaluateNN(input');
            maxV = 8;
            r           = output(1) * maxV;
            relTheta    = output(2) * 2*pi() - pi();
            L_i         = r * (([-1 0 0] * obj.arena.rotMat(-obj.heading(obj.arena.t,1))) * obj.arena.rotMat(relTheta));
            
            d_i     = [0 0 0];                                                  % Dissipative energy
            if obj.arena.t > 1
                d_i = -eps*(obj.pos(obj.arena.t-1,:)-obj.pos(obj.arena.t,:));   % Calculate dissipative energy
            end
            u_d = g_i + L_i + d_i;                  % Sum to find u_d
            obj.u_d_decom.g(obj.arena.t,:) = g_i;   % Save to array for plotting
            obj.u_d_decom.L(obj.arena.t,:) = L_i;   % Save to array for plotting
            obj.u_d_decom.d(obj.arena.t,:) = d_i;   % Save to array for plotting
            v_d = u_d;                              % Convert u_d to v_d
        end
    end
    
end
