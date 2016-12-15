classdef Agent_polynomial < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Agent_polynomial(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
        end
        
        function y = local_interaction(obj,x)
            sigma   = obj.seperation_range;
            y       = 1 ./ x .* sum(repmat(sigma ./ x',1,length(obj.genome)-2).^(0:(length(obj.genome)-3)) .* obj.genome(3:end),2)';
        end
    end
    
end