function [x,fval,exitFlag,output,population,scores] = galincon(FitnessFcn,GenomeLength, ...
    Aineq,bineq,Aeq,beq,lb,ub,options,output,Iterate)
%GALINCON Genetic algorithm linear constrained solver.
%   GALINCON solves problems of the form:
%           min F(X)    subject to:      A*x <= b
%            X                          Aeq*x = beq
%                                      LB <= X <= UB
%   Private function to GA

%   Copyright 2005-2015 The MathWorks, Inc.

% Initialize output args
x = []; fval = []; exitFlag = [];
LinearConstr = options.LinearConstr;

% Create initial state: population, scores, status data
state = makeState(GenomeLength,FitnessFcn,Iterate,output.problemtype,options);
% Determine who is the caller
callStack = dbstack;
[~,caller] = fileparts(callStack(2).file);

% Set state for plot and output functions (only gacon will have
% 'interrupt' state)
if ~strcmp(caller,'gacon')
    currentState = 'init';
else
    currentState = 'interrupt';
end
% Give the plot/output Fcns a chance to do any initialization they need.
global alt_gads;
if alt_gads
    state = alt_gadsplot(options,state,currentState,'Genetic Algorithm');
else
    state = gadsplot(options,state,currentState,'Genetic Algorithm');
end
[state,options] = gaoutput(FitnessFcn,options,state,currentState);

% Setup display header
if  options.Verbosity > 1
    fprintf('\n                               Best           Mean      Stall\n');
    fprintf('Generation      f-count        f(x)           f(x)    Generations\n');
end

% Set state for plot and output functions (only gacon will have
% 'interrupt' state)
if ~strcmp(caller,'gacon')
    currentState = 'iter';
else
    currentState = 'interrupt';
end
% Run the main loop until some termination condition becomes true
while isempty(exitFlag)
    state.Generation = state.Generation + 1;
    % Repeat for each subpopulation (element of the populationSize vector)
    offset = 0;
    totalPop = options.PopulationSize;
    % Each sub-population loop
    for pop = 1:length(totalPop)
        populationSize =  totalPop(pop);
        thisPopulation = 1 + (offset:(offset + populationSize - 1));
        population = state.Population(thisPopulation,:);
        score = state.Score( thisPopulation );
        % Empty population is also possible
        if isempty(thisPopulation)
            continue;
        end
        [score,population,state] = stepGA(score,population,options,state,GenomeLength,FitnessFcn);
        % Store the results for this sub-population
        state.Population(thisPopulation,:) = population;
        state.Score(thisPopulation) = score;
        offset = offset + populationSize;
    end

    % Remember the best score
    best = min(state.Score);
    generation = state.Generation;
    state.Best(generation) = best;
    % Keep track of improvement in the best
    if((generation > 1) && isfinite(best))
        if(state.Best(generation-1) > best)
            state.LastImprovement = generation;
            state.LastImprovementTime = tic;
        end
    end
    % Do any migration
    state = migrate(FitnessFcn,GenomeLength,options,state);
    % Update the Output
    if alt_gads
        state = alt_gadsplot(options,state,currentState,'Genetic Algorithm');
    else
        state = gadsplot(options,state,currentState,'Genetic Algorithm');
    end
    [state,options,optchanged] = gaoutput(FitnessFcn,options,state,currentState);
    if optchanged
        options.LinearConstr = LinearConstr;
    end
    % Check to see if any stopping criteria have been met
    [exitFlag,output.message] = isItTimeToStop(options,state);
end % End while loop

% Find and return the best solution
[fval,best] = min(state.Score);
x = state.Population(best,:);

% Update output structure
output.generations = state.Generation;
output.funccount   = state.FunEval;
output.maxconstraint = norm([Aeq*x'-beq; max([Aineq*x' - bineq;x' - ub(:);lb(:) - x'],0)],Inf);
population = state.Population;
scores = state.Score;

% Call hybrid function
if ~isempty(options.HybridFcn)
    [x,fval] = callHybridFunction;
end
% Set state for plot and output functions (only gacon will have
% 'interrupt' state)
if ~strcmp(caller,'gacon')
    currentState = 'done';
else
    currentState = 'interrupt';
end
% give the Output functions a chance to finish up
if alt_gads
    alt_gadsplot(options,state,currentState,'Genetic Algorithm');
else
    gadsplot(options,state,currentState,'Genetic Algorithm');
end
gaoutput(FitnessFcn,options,state,currentState);

%-----------------------------------------------------------------
% Hybrid function
    function [xhybrid,fhybrid] = callHybridFunction
        xhybrid = x;
        fhybrid = fval;
        % Who is the hybrid function
        if isa(options.HybridFcn,'function_handle')
            hfunc = func2str(options.HybridFcn);
        else
            hfunc = options.HybridFcn;
        end
        % Inform about hybrid scheme
        if options.Verbosity > 1
            fprintf('%s%s%s\n','Switching to the hybrid optimization algorithm (',upper(hfunc),').');
        end
        % Create functions handle to be passed to hybrid function
        FitnessHybridFcn = @(x) FitnessFcn(x,options.FitnessFcnArgs{:});
        ConstrHybridFcn = [];

        [x_temp,f_temp,funccount,theMessage,conviol_temp] = ...
            callHybrid(hfunc,FitnessHybridFcn,x,options.HybridFcnArgs,Aineq,bineq,Aeq,beq,lb,ub,ConstrHybridFcn);
        output.funccount = output.funccount + funccount;
        output.message   = sprintf([output.message '\n', theMessage '\n']);
        hybridPointFeasible = isHybridPointFeasible(conviol_temp, ...
            hfunc, options.HybridFcnArgs{:});
        % We have to recheck whether the final point returned from GA is
        % feasible because GA asserts that the final point is always
        % feasible. If a user supplies their own operators, this may not be
        % the case. We could replace this code if output.maxconstraint
        % reflects the true constraint violation.
        tol = max(sqrt(eps),options.TolCon);
        gaPointFeasible = isTrialFeasible(x(:), Aineq, bineq, ...
                                          Aeq, beq, lb, ub, tol);
        if hybridPointFeasible && (~gaPointFeasible || f_temp < fhybrid)
            fhybrid = f_temp;
            xhybrid = x_temp;
        end
        % Inform about hybrid scheme termination
        if  options.Verbosity > 1
            fprintf('%s%s\n',upper(hfunc), ' terminated.');
        end
    end % End of callHybridFunction
end  % End of galincon
