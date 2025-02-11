%% x-line tracking
xline                  = struct(...
                            'name',     'x-line-tracking',...
                            'type',     'bucket'                    ,...
                            'c_pos',    [0 0 10]                    ,...
                            'c_fun', @(t) [50*sin(1/50*3*t) 0 10],...
                            'varargin', struct(                     ...
                                'v_max',         3,...
                                'v_min',         1.0...
                            )...
                          );
%% Bucket
buck                    = struct(...
                            'name',     'bucket',...
                            'type',     'bucket'                    ,...
                            'c_pos',    [0 0 10]                    ,...
                            'varargin', struct(                     ...
                                'v_max',         3,...
                                'v_min',         0.5...
                            )...
                          );
%% Two point circles
two_point_circ(1)        = struct(...
                            'name', 'two-point-circles',...
                            'type',     'point'                    ,...
                            'c_pos',    [-20 0 10]                    ,...
                            'c_fun', @(t) [-10*cos(1/10*3*t)-10 10*sin(1/10*3*t) 10],...
                            'varargin', struct('v_max', 3)...
                          );
two_point_circ(2)        = struct(...
                            'name', 'two-point-circles',...
                            'type',     'point'                    ,...
                            'c_pos',    [20 0 10]                    ,...
                            'c_fun', @(t) [10*cos(1/10*3*t)+10 10*sin(1/10*3*t) 10],...
                            'varargin', struct('v_max', 3)...
                          );
%% Two bucket circles
two_buck_circ(1)        = struct(...
                            'name', 'two-bucket-circles',...
                            'type',     'bucket'                    ,...
                            'c_pos',    [-35 30 10]                    ,...
                            'c_fun', @(t) [-30*cos(1/30*3.5*t + 0.5*pi())-25 30*sin(1/30*3.5*t + 0.5*pi()) 10],...
                            'varargin', struct(                     ...
                                'v_max',         3.5,...
                                'v_min',         0.5...
                            )...
                          );
two_buck_circ(2)        = struct(...
                            'name', 'two-bucket-circles',...
                            'type',     'bucket'                    ,...
                            'c_pos',    [35 30 10]                    ,...
                            'c_fun', @(t) [30*cos(1/30*3.5*t + 0.5*pi())+25 30*sin(1/30*3.5*t + 0.5*pi()) 10],...
                            'varargin', struct(                     ...
                                'v_max',         3.5,...
                                'v_min',         0.5...
                            )...
                          );
%% Two circles
two_circ(1)            = struct(                                   ...
                            'name', 'two-circles',...
                            'type',     'circle'                    ,...
                            'c_pos',    [-8 0 10]                    ,...
                            'varargin', struct(                     ...
                                'radius',           7.5               ,...
                                'v_max',            3    ,...
                                'direction',        1               ,...
                                'bandwidth_gain',   0.01            ,...
                                'spiral_gain',      6               ...
                            )                                       ...
                          );   
two_circ(2)            = struct(                                   ...
                            'name', 'two-circles',...
                            'type',     'circle'                    ,...
                            'c_pos',    [8 0 10]                    ,...
                            'varargin', struct(                     ...
                                'radius',           7.5               ,...
                                'v_max',            3    ,...
                                'direction',        -1              ,...
                                'bandwidth_gain',   0.01            ,...
                                'spiral_gain',      6               ...
                            )                                       ...
                          ); 
%% Large ellipse
lcirc                  = struct(...
                            'name',     'large-circle',...
                            'type',     'bucket'                    ,...
                            'c_pos',    [0 50 10]                    ,...
                            'c_fun', @(t) [50*sin(1/50*3.5*t) 50*cos(1/50*3.5*t) 10],...
                            'varargin', struct(                     ...
                                'v_max',         3.5,...
                                'v_min',         0.5...
                            )...
                          );