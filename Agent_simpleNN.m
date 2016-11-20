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
            y = (x - obj.collision_range + obj.seperation_range)/(obj.collision_range + obj.seperation_range);
        end
        
        function y = local_interaction(obj,x)
            %y = obj.v_max * obj.net(obj.nnNormI(x));
            switch(obj.net.numLayers)
                case 2 % One hidden layer
                    y = tansig(obj.net.b{2} + obj.net.LW{2} * radbas(obj.net.IW{1} * obj.nnNormI(x) + obj.net.b{1}));
                case 3 % Two hidden layers
                    y = tansig(obj.net.b{3} + obj.net.LW{3,2} * radbas(obj.net.LW{2,1} * radbas(obj.net.IW{1} * obj.nnNormI(x) + obj.net.b{1}) + obj.net.b{2}));
                otherwise
                    y = zeros(1,length(x));
                    disp('ERROR unrecognized nr of layers');
            end
            y = obj.v_max * obj.a * y;
        end
    end
    
end