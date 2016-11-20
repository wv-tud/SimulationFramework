classdef Agent_sinusoid < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Agent_sinusoid(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
%             if ~isa(obj.g_fun,'function_handle')
%                 a           = obj.v_max*obj.dt/(((obj.swarmSize*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
%                 obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
%             end
        end

        function y = local_interaction(obj,x)
            y       = obj.genome(2) .* ones(size(x));
            sigma   = obj.seperation_range + obj.collision_range;
            for l=1:(length(obj.genome)-2)/3
                y = y + obj.genome(3+(l-1)*3) .* sin(2*pi()*obj.genome(4+(l-1)*3) * (x/sigma) + 2*pi()*obj.genome(5+(l-1)*3));
            end
        end
    end
end

