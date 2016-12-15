classdef Agent_sinusoid < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Agent_sinusoid(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
        end

        function y = local_interaction(obj,x)
            y       = obj.genome(3) .* ones(size(x));
            sigma   = obj.seperation_range;
            for l=1:(length(obj.genome)-3)/3
                y = y + obj.genome(4+(l-1)*3) .* sin(2*pi()*obj.genome(5+(l-1)*3) * (x/sigma) + 2*pi()*obj.genome(6+(l-1)*3));
            end
        end
    end
end

