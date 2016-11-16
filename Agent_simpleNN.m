classdef Agent_simpleNN < Agent
    %SIMPLENNAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties          
    end
    
    methods
        function obj = Agent_simpleNN(arena,id,pos,head)
            obj     = obj@Agent(arena,id,pos,head);
        end

        function v_d = calculate_vd(obj)
            v_d = obj.evaluateNN();
        end
        
        function v_d = evaluateNN(obj)
            eps     = obj.genome(1);
            q_i     = obj.pos' - obj.c_pos;                 % Vector of current position and c
            g_i     = obj.global_interaction(q_i);
            L_i     = [0 0 0];                                                  % Lattice formation
            if ~isempty(obj.neighbours{obj.t})
                vL_i    = zeros(size(obj.neighbours{obj.t},1),3);
                vq_ijn  = zeros(size(obj.neighbours{obj.t},1),1);
                for j=1:size(obj.neighbours{obj.t},1)
                    q_ij    = q_i - (obj.neighbours{obj.t}(j,3:5) - obj.c_pos);                          % Relative vector between agent i and j
                    q_ijn   = sqrt(q_ij(1)^2+q_ij(2)^2+q_ij(3)^2);                                                  % Normalised relative vector
                    if q_ijn > 0
                        vq_ijn(j) = q_ijn;
                        vL_i(j,:) = [q_ij(1)/q_ijn q_ij(2)/q_ijn 0]; % Calculate Lattice formation  
                    end
                end
                nnL_i   = obj.local_interaction(vq_ijn')';
                L_i     = mean(nnL_i .* vL_i,1); % Average over nr. of agents    
            end
            u_d = g_i + L_i;                  % Sum to find u_d
            u_d_n = sum(u_d.^2);
            if u_d_n > obj.v_max
                g_i = g_i ./ u_d_n * obj.v_max;
                L_i = L_i ./ u_d_n * obj.v_max;
                u_d = g_i + L_i;
            end
            if obj.t > 1
                d_i = -eps*(L_i+g_i - (obj.u_d_decom.g(obj.t-1,:)+obj.u_d_decom.L(obj.t-1,:)+obj.u_d_decom.d(obj.t-1,:)));   % Calculate dissipative energy
            else
                d_i = -eps*(L_i+g_i);
            end
            v_d = u_d + d_i;
            obj.u_d_decom.g(obj.t,:) = g_i;   % Save to array for plotting
            obj.u_d_decom.L(obj.t,:) = L_i;   % Save to array for plotting
            obj.u_d_decom.d(obj.t,:) = d_i;   % Save to array for plotting
        end
        
        function y = nnNormI(obj,x)
            y = (x - (obj.collision_range + obj.seperation_range))./(obj.cam_range);
        end
        
        function y = local_interaction(obj,x)
            %y = obj.v_max * obj.net(obj.nnNormI(x));
            switch(obj.net.numLayers)
                case 2 % One hidden layer
                    y = obj.net.b{2} + obj.net.LW{2} * tansig(obj.net.IW{1} * obj.nnNormI(x) + obj.net.b{1});
                case 3 % Two hidden layers
                    y = obj.net.b{3} + obj.net.LW{3,2} * tansig(obj.net.LW{2,1} * tansig(obj.net.IW{1} * obj.nnNormI(x) + obj.net.b{1}) + obj.net.b{2});
                otherwise
                    y = zeros(1,length(x));
                    disp('ERROR unrecognized nr of layers');
            end
            y = obj.v_max * y;
        end
    end
    
end
