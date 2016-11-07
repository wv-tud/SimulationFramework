classdef sinusoidAgent < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % PolyAgent properties
        seperation_range    = 1;                    % Ideal seperation between agents
        g_fun               = [];                   % Function handle for global pinciroli attractor  
        g_fun2              = [];                   % Second function handle for conditional functions
        g_cond              = 0;                    % Condition for second function handle
    end
    
    methods
        function obj = sinusoidAgent(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
%             if ~isa(obj.g_fun,'function_handle')
%                 a           = obj.v_max*obj.dt/(((obj.swarmSize*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
%                 obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
%             end
        end
        
        function v_d = calculate_vd(obj)
            v_d = obj.sinusoidCalculation();
        end
        
        function v_d = sinusoidCalculation(obj)
            sigma   = obj.seperation_range + obj.collision_range;
            eps     = obj.genome(1);
            q_i     = obj.pos - obj.c_pos;                 % Vector of current position and c
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
            L_i     = [0 0 0];                                                  % Lattice formation
            d_i     = [0 0 0];                                                  % Dissipative energy
            if ~isempty(obj.neighbours{obj.t})
                for j=1:size(obj.neighbours{obj.t},1)
                    q_ij    = q_i-(obj.neighbours{obj.t}(j,3:5) - obj.c_pos);                          % Relative vector between agent i and j
                    q_ijn   = sqrt(q_ij(1)^2+q_ij(2)^2+q_ij(3)^2);                                                  % Normalised relative vector
                    if q_ijn > 0
                        L_i = L_i + obj.genome(2);
                        for l=1:(length(obj.genome)-2)/3
                            L_i = L_i + obj.genome(3+(l-1)*3) * sin(2*pi()*obj.genome(4+(l-1)*3) * (q_ijn/sigma) + 2*pi()*obj.genome(5+(l-1)*3)) * [q_ij(1)/q_ijn q_ij(2)/q_ijn 0];
                        end
                    end
                end
                L_i = L_i / length(obj.neighbours{obj.t});     % Average over nr. of agents
            end
            if obj.t > 1
                d_i = -eps*(L_i+g_i - (obj.u_d_decom.g(obj.t-1,:)+obj.u_d_decom.L(obj.t-1,:)+obj.u_d_decom.d(obj.t-1,:)));   % Calculate dissipative energy
            end
            u_d = g_i + L_i + d_i;                  % Sum to find u_d
            obj.u_d_decom.g(obj.t,:) = g_i;   % Save to array for plotting
            obj.u_d_decom.L(obj.t,:) = L_i;   % Save to array for plotting
            obj.u_d_decom.d(obj.t,:) = d_i;   % Save to array for plotting
            v_d = u_d;                              % Convert u_d to v_d
        end
        
        function y = local_interaction(obj,x)
            y = obj.genome(1);
            sigma   = obj.seperation_range + obj.collision_range;
            for l=1:(length(obj.genome)-1)/3
                y = y + obj.genome(2+(l-1)*3) * sin(2*pi()*obj.genome(3+(l-1)*3) * (x/sigma) + 2*pi()*obj.genome(4+(l-1)*3));
            end
        end
        
        function plotAgentFunction(obj,figid)
            figure(figid);
            sigma   = obj.seperation_range + obj.collision_range;
            x=0:0.01:5;
            l = zeros(1,length(x));
            for i=1:length(x)
                for s=1:(length(obj.genome)-1)/3
                    l(i) = l(i) + obj.genome(2+(s-1)*3) * sin(2*pi()*obj.genome(3+(s-1)*3) * (x(i)/sigma) + 2*pi()*obj.genome(4+(s-1)*3));
                end
            end
            subplot(1,2,1);
            plot(x,l);
            axis([0 x(end) 0 2*obj.v_max]);
            xlabel('Distance from agent [m]');
            ylabel('Polynomial response [m/s]');
            subplot(1,2,2);
            plot(1:length(obj.genome),obj.genome,'o');
            xlabel('Polynomial order');
            ylabel('Coefficient');
        end
    end
    
end

