runTrainSim      = 1;
writeTrainVideos = 0;
generateNnData   = 1;
trainNetwork     = 1;
runSim           = 1;

if runTrainSim == 1
    close all hidden;
    clear uArena;
    names           = {'cyberzooCW' 'cyberzooCCW' 'cyberzooBucket'};
    add_args        = {0 0 0 0};
    v_max           = 3;
    % Simulate
    simTime     = 900;
    simAgents   = 8;
    SimParallel;
end
if writeTrainVideos == 1
    createVideos;
end
% Generate NN data
fprintf('\n\nStarting on dataset\n');
if generateNnData == 1
    clear input output;
    generateNNdataset;
end
% Train the network
if trainNetwork == 1
    hiddenLayerSize = 50;
    % Choose a Training Function
    % For a list of all training functions type: help nntrain
    % 'trainlm' is usually fastest.
    % 'trainbr' takes longer but may be better for challenging problems.
    % 'trainscg' uses less memory. Suitable in low memory situations.
    trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.
    trainNN;
end
% Run simulated network
if runSim == 1
    netAgents = 8;
    clear uArena;
    names       = {'cyberzooBucket' 'cyberzooCW' 'cyberzooCCW'};
    add_args    = {0 0 0 0};
    v_max       = 4;
    writeVideo  = 1;
    simTime     = 60;
    simAgents   = 8;
    netAgents   = 4;
    SimParallel;
    
    if writeVideo == 1
        createVideos;
    end
end