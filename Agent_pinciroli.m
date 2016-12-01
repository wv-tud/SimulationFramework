classdef Agent_pinciroli < Agent
    %PINCIROLIAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Agent_pinciroli(arena,id,pos,head)
            obj = obj@Agent(arena,id,pos,head);
%             if ~isa(obj.g_fun,'function_handle')
%                 a           = obj.v_max*obj.dt/(((obj.swarmSize*(obj.collision_range+obj.seperation_range)^2*sqrt(3)/2)/pi()));
%                 obj.g_fun   = @(varargin) min(a*norm(varargin{2}).^2,varargin{3})*varargin{2}./norm(varargin{2}).*[-1 -1 0];
%             end
        end
        
        function y = local_interaction(obj,x)
            sigma   = (obj.seperation_range + obj.collision_range) ./ x;
            y       = 12 * obj.genome(3) ./ x .* (sigma.^12 - sigma.^6);
        end
    end
    
end