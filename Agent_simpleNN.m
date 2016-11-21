classdef Agent_simpleNN < Agent
    %SIMPLENNAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties   
        a       = 1; % Weight for neural output
    end
    
    methods
        function obj = Agent_simpleNN(arena,id,pos,head)
            obj     = obj@Agent(arena,id,pos,head);
        end

        function y = nnNormI(obj,x)
            y = (x - obj.collision_range + obj.seperation_range)./(obj.collision_range + obj.seperation_range);
        end
        
        function y = local_interaction(obj,x)
            y = obj.v_max * obj.a * tansig(obj.net.OB + obj.net.LW * radbas(obj.net.IW' * obj.nnNormI(x) + obj.net.IB'));
        end
    end
    
end