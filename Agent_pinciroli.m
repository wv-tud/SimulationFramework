classdef Agent_pinciroli < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Agent_pinciroli(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
        end
        
        function y = local_interaction(obj,x)
            sigma   = (obj.seperation_range) ./ x;
            y       = 12 * obj.genome(3) ./ x .* (sigma.^12 - sigma.^6);
        end
    end
    
end