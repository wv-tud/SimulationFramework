function state = alt_gadsplot(options,optimvalues,flag,fname)
%GADSPLOT Helper function that manages the plot functions.
%   STOP = GADSPLOT(options,optimvalues,flag,fname) runs
%   each of the plot functions.
%
%   This function is private to simulanneal, patternsearch, ga, gamultiobj.

%   Copyright 2006-2015 The MathWorks, Inc.

persistent plotState plotDrawNowLast

solver = matlab.lang.makeValidName(fname);
%varString = @(x) inputname(1);
gadsSolverState = [solver,'State'];

% Different for GA and other solvers
if strcmpi(fname,'genetic algorithm')
  state = optimvalues;
  currentIter = optimvalues.Generation;
elseif any(strcmpi(fname, {'multistart', 'globalsearch'}))
  % Note that there are no spaces in the fname for MultiStart and
  % GlobalSearch, as this is how these solvers are referred to in the
  % product documentation.
  state = false;
  currentIter = optimvalues.localrunindex;
  % GlobalSearch and MultiStart do not support PlotInterval and
  % PlotFcnsArgs. Set them to sensible defaults.
  options.PlotInterval = 1;
  options.PlotFcnsArgs = repmat({{}}, 1, length(options.PlotFcns));
elseif strcmpi(fname, 'particleswarm')
  state = false;
  currentIter = optimvalues.iteration;  
  % particleswarm does not support PlotInterval and
  % PlotFcnsArgs. Set them to sensible defaults.
  options.PlotInterval = 1;
  options.PlotFcnsArgs = repmat({{}}, 1, length(options.PlotFcns));  
else
  state = false;
  currentIter = optimvalues.iteration;
end

if ~strcmpi(flag, 'done') && (rem(currentIter,options.PlotInterval) ~=0)
  return;
end

functions = options.PlotFcns;
fcnargs = options.PlotFcnsArgs;

if isempty(functions) || ...
    (any(strcmpi(flag,{'interrupt','done'})) && isempty(findobj(0,'Type','figure','name',fname)))
  return;
end
% It is always safe to reset the state of the solver state when plots are
% called with 'init' flag.
if strcmpi(flag,'init')
  setappdata(0,gadsSolverState,'')
end
% Check if 'stop' was requested before coming to this plot function driver
if(strcmpi('stop',getappdata(0,gadsSolverState)))
  % Different for GA and other solvers
  if strcmpi(fname,'genetic algorithm')
    state.StopFlag = 'stop requested from plot function.';
  else
    state= true;
  end
  setappdata(0,gadsSolverState,'')
  return;
end

functions = removeDup(functions);
fig = findobj(0,'type','figure','name',fname);
% Called with 'init' flag or the figure is not present
if (strcmp(flag,'init') || isempty(fig)) && ...
    ~strcmp(flag,'interrupt')

  if isempty(fig)
    fig = figure('visible','off');
    try
       position =  plotState.(solver).position; 
    catch
       position =  [];
    end
    if ~isempty(position) && ~strcmpi(get(fig,'WindowStyle'),'docked')
      set(fig,'Position',position);
    end
  end
  set(0,'CurrentFigure',fig);
  clf;
  set(fig,'numbertitle','off','name',fname, 'userdata',[]);
  % Initialize the persistent variables
  plotState.(solver) = struct(); plotDrawNowLast = tic;
  axhandle = []; 
  [plotNames, args, plotNo, isNew] = updatelist([], [], [], ...
    functions, fcnargs, fig, 'init');
  
  if ~isdeployed
    % optimtool may not be present when this function is called from deployed apps.
    optimtoolGui = javaMethodEDT('getOptimGUI','com.mathworks.toolbox.optim.OptimGUI');
    isGUIOpen = ~isempty(optimtoolGui);
  % Check the state of the GUI. getRunMode == 1 when the GUI is running/paused.    
    if isGUIOpen
      isGUIRunning = (javaMethodEDT('getRunMode',optimtoolGui) == 1);
    else
      isGUIRunning = false;
    end
    buttonsOK = ~(isGUIOpen && isGUIRunning);
  else
    buttonsOK = true;
  end
  if buttonsOK
    pObj = uicontrol('string','Pause', ... 
      'Position',[60 5 50 20],'callback',{@buttonPauseContinue,gadsSolverState});
    uicontrol('string','Stop', ... 
      'Position',[2 5 50 20],'callback',{@buttonStop,pObj,gadsSolverState});
  end
  set(fig,'CloseRequestFcn',{@beforeClose,gadsSolverState});
  setappdata(0,gadsSolverState,'')
  set(gcf,'visible','on')
  shg
else
  plotNo =  plotState.(solver).plotNo;
  plotNames =  plotState.(solver).plotNames;
  args =  plotState.(solver).args;
  isNew =  plotState.(solver).isNew;
  axhandle =  plotState.(solver).axhandle;
end
% Determine the layout size in the figure
[rows, cols] = plotLayout(functions);

% Set the current figure to fig
set(0,'CurrentFigure',fig);

if change_runtime(functions,plotNames)
  [plotNames, args, plotNo, isNew] = updatelist(plotNames, plotNo, axhandle, ...
    functions, fcnargs, fig, '');
  figure(fig);
end

if all(isNew)
  mouseaction([],[],[],[],'init');
end

% Call each plot function
for i = 1:length(plotNames)
  if isNew(i)
    axhandle(i) = subplot(rows,cols,plotNo(i));
    cmenu(i) = uicontextmenu;
    set(axhandle(i),'UIContextMenu', cmenu(i));
    % Provide a uicontext menu item to open the axes in a new figure
    % window
    cmenuCallback = {@mouseaction,axhandle(i),plotNames{i},'add'};
    uimenu(cmenu(i),'Label', 'Open this plot in a new window', 'Callback', cmenuCallback);
    % Do not delete the axis (which is the default settings)
    set(axhandle(i),'NextPlot','add','Tag',func2str(plotNames{i}));
    [state,optimvalues] = callOnePlotFcn(fname,plotNames{i},state,options.OutputPlotFcnOptions,optimvalues,'init',args{i}{:});
    isNew(i)= false;
    % If in the middle of itertions, call with the regular 'flag' too.
    if ~strcmpi(flag,'init')
      [state,optimvalues] = callOnePlotFcn(fname,plotNames{i},state,options.OutputPlotFcnOptions,optimvalues,flag,args{i}{:});
    end
  else
    [state,optimvalues] = callOnePlotFcn(fname,plotNames{i},state,options.OutputPlotFcnOptions,optimvalues,flag,args{i}{:});
  end
end

% Call plot functions whose UIContextMenu has been called
state = gadsplotagain(fname,plotNames,args,options.OutputPlotFcnOptions,optimvalues,state,flag);

if strcmpi(flag,'done') % reset the closerequest function
  set(fig,'CloseRequestFcn','closereq');
  if isappdata(0,gadsSolverState)
    rmappdata(0,gadsSolverState);
  end
end

% Call drawnow at most 1/30 seconds later
if toc(plotDrawNowLast) > 1/30
  drawnow
  plotDrawNowLast = tic;
end
plotState.(solver).plotNo = plotNo;
plotState.(solver).plotNames = plotNames;
plotState.(solver).args = args;
plotState.(solver).isNew = isNew;
plotState.(solver).axhandle = axhandle;
  
% Check if the figure is still alive or 'stop' is requested
if isempty(findobj(0,'Type','figure','name',fname)) || ...
    (strcmpi('stop',getappdata(0,gadsSolverState)))
  % Different for GA and other solvers
  if strcmpi(fname,'genetic algorithm')
    state.StopFlag = 'stop requested from plot function.';
  else
    state= true;
  end
  setappdata(0,gadsSolverState,'')
  return;
end

% Remember the position
plotState.(solver).position = get(fig,'Position');

%-------------------------------------------------------
%CALLONEPLOTFCN Calls a plot function
%-------------------------------------------------------
function [state,optimvalues] = callOnePlotFcn(fname,plotfcn,state,varargin)
%
pause
optimvalues = varargin{2};
switch lower(strtrim(fname))
  case {'pattern search','multistart','globalsearch','particleswarm'}
    state = plotfcn(varargin{2:end}) || state;
  case {'simulated annealing'}
    state = plotfcn(varargin{1:end}) || state;
  case 'genetic algorithm'
    optimvalues = plotfcn(varargin{1:end});
    state = optimvalues;
end
%-------------------------------------------------------
%UPDATELIST updates the function list and plot numbers
%-------------------------------------------------------
function [plotNames, fcnArgs, plotNo, isNew] = updatelist(plotNames,plotNo,axhandle,functions,args,fig,flag)

if strcmpi(flag,'init')
  plotNames = functions;
  plotNo   = 1:length(functions);
  isNew = true(length(plotNames),1);
  fcnArgs = args;
  return;
end

% Determine the layout size in the figure
[rows, cols] = plotLayout(functions);
% Wwhat was the layout size before
[rows1, cols1] = plotLayout(plotNames);

set(0,'CurrentFigure',fig);
fcnArgs = cell(1,length(plotNames));
isNew = false(length(plotNames),1);
to_delete = false(length(plotNames),1);
%Check if any of plotNames is not in functions;remove such entries
for i = 1:length(plotNames)
  [found, index] = foundfunc(plotNames{i},functions);
  if ~found
    delete_this = subplot(rows1,cols1,plotNo(i));
    delete(delete_this);
    to_delete(i) = true;
  else
    fcnArgs(i) = args(index(1));
  end
end
%delete the plot names which are not in functions
plotNames(to_delete) = []; plotNo(to_delete) = [];
isNew(to_delete) = []; fcnArgs(to_delete) = [];
axhandle(to_delete) = [];

%Now, add all new entries of functions in plotNames
for i = 1:length(functions)
  found = foundfunc(functions{i},plotNames);
  if ~found
    plotNames(end+1) = functions(i);
    fcnArgs(end+1) = args(i);
    isNew(end+1) = true;
    plotNo(end+1) = 0;
    axhandle(end+1) = 0;
  end
end

%Determine the plot numbers
if rows1 == rows && cols1 == cols
  %Binary search and replacement
  for i = 1:length(plotNames)
    if plotNo(i) == 0
      for j = 1:rows*cols
        if ~any(j == plotNo)
          plotNo(i) = j;
        end
      end
    end
  end
elseif rows1 > rows || cols1 > cols
  %find position of all existing axes and shift them
  for i = 1:length(plotNames)
    if plotNo(i) ~=0
      myaxis = axhandle(i);
      plotNo(i) = i;
    else
      plotNo(i) = i;
      myaxis = subplot(rows,cols,plotNo(i));
    end
    subplot(rows,cols,plotNo(i),myaxis);
  end
elseif rows1 < rows || cols1 < cols
  for i = 1:length(plotNames)
    if plotNo(i) ~=0
      myaxis = axhandle(i);
      plotNo(i) = i;
    else
      plotNo(i) = i;
      myaxis = subplot(rows,cols,plotNo(i));
    end
    subplot(rows,cols,plotNo(i),myaxis);
  end
  
end

%-----------------------------------------------------------
%CHANGE_RUNTIME return a boolean if two cell arrays are same or not
%-----------------------------------------------------------
function bool = change_runtime(functions,plotNames)
bool = false;
for i = 1:length(functions)
  if ~foundfunc(functions{i},plotNames)
    bool = true;
  end
end

for i = 1:length(plotNames)
  if ~foundfunc(plotNames{i},functions)
    bool = true;
  end
end

%-----------------------------------------------------------
%REMOVEDUP remove the duplicate entries in a cell array of function handle
%-----------------------------------------------------------
function functions = removeDup(functions)
i = 1;
while i <= length(functions)
  [found,index] = foundfunc(functions{i},functions);
  if found
    functions(index(1:end-1)) = [];
  end
  i = i+1;
end

%-------------------------------------------------------------------------
%FOUNDFUNC Finds if STR is in FUNCNAMES, returns a boolean and index
%-------------------------------------------------------------------------
function [bool,index] = foundfunc(str,funcNames)

bool = false;
index = 0;
for i = 1:length(funcNames)
  if strcmpi(func2str(str),func2str(funcNames{i}))
    bool = true;
    if nargout > 1
      index(end+1) = i;
    end
  end
end
index(1) = [];
%-----------------------------------------------------------
% STOP callback
%-----------------------------------------------------------
function buttonStop(~,~,pObj,gadsSolverState)
% Determine the length of stack. If length is one then we don't need to
% perform the callback action
callStack = dbstack;
if length(callStack) == 1
  return;
end
setappdata(0,gadsSolverState,'stop');
% Make sure that Pause button have 'Pause' string
if ~isempty(pObj)
  set(pObj,'String','Pause');
end


%-----------------------------------------------------------
% PAUSE/CONTINUE button callback
%-----------------------------------------------------------
function buttonPauseContinue(hObj,~,gadsSolverState)
% Determine the length of stack. If length is one then we don't need to
% perform the callback action
callStack = dbstack;
if length(callStack) == 1
  return;
end
if isempty(getappdata(0,gadsSolverState))
  setappdata(0,gadsSolverState,'pause');
  set(hObj,'String','Resume');
  % Wait for hObj to change its String property
  waitfor(hObj,'String');
  if isempty(findobj(0,'Type','uicontrol','string','Pause')) % Figure is deleted
    setappdata(0,gadsSolverState,'');
  end
else
  setappdata(0,gadsSolverState,'');
  set(hObj,'String','Pause');
end

%-----------------------------------------------------------
%UICONTEXTMENU Callback maintain a list of all the plots that need to be
%plotted twice
%-----------------------------------------------------------
function [done, func] = mouseaction(obj,eventdata,axes_handle,Name,what)
persistent list
done = false;
func = [];
% Determine the length of stack. If length is one then need to open a new
% figure with axes copied from the current object
callStack = dbstack;
if length(callStack) ==1 % The solver has stopped
  % "Name" is guaranteed to be a function handle here, so we can call
  % func2str to get the function name for the figure window.
  newFigName = func2str(Name);
  fig = findobj(0,'type','figure','name',newFigName);
  if isempty(fig) % Create a new figure
    fig = figure('numbertitle','off','name',newFigName);
  end
  set(0,'CurrentFigure',fig); clf;
  % Get the position of new axes (to be created)
  tempaxis = axes('visible','off');
  axisPosition = get(tempaxis,'Position');
  delete(tempaxis);
  % Copy the axes to the new figure
  parent = get(axes_handle,'parent');
  copiedPlot = copyobj(axes_handle,parent);
  set(copiedPlot,'parent',fig,'position',axisPosition);
  figure(fig);
  return;
end
% Switch case to maintain the list of functions that need to be called
% twice (when 'add' or 'remove' is called
switch lower(what)
  case 'length'
    if ~isempty(list)
      done = true;
    else
      done = false;
    end
  case 'init'
    list = [];
    done = false;
  case 'add'
    if isempty(list)
      list{1} = Name;
    elseif ~foundfunc(Name,list);
      list{end+1} = Name;
      done = true;
    end
  case 'remove'
    if ~isempty(list)
      [found,index] = foundfunc(Name,list);
      if found
        list(index) = [];
        done = true;
      end
    end
end
func = list;
%-----------------------------------------------------------
%GAPLOTAGAIN plots all the functions whose ButtondownFcn has been
%called.
%-----------------------------------------------------------
function state = gadsplotagain(solverName,plotNames,args,options,optimvalues,state,flag)

[foundAny, func] = mouseaction([],[],[],[],'length');
if ~foundAny
  return;
else
  to_delete = false(1,length(func));
  fcnArgs = cell(1,length(func));
  for i = 1:length(func)
    [found, index] = foundfunc(func{i},plotNames);
    if ~found
      to_delete(i) = true;
    else
      fcnArgs(i) = args(index(1));
    end
  end
  % Delete the plot names which are not in functions
  func(to_delete) = [];  fcnArgs(to_delete) = [];
end

% call each plot function
for i = 1:length(func)
  fname = func2str(func{i});
  fig = findobj(0,'type','figure','name',fname);
  % Called with 'init' flag or the figure is not present
  if isempty(fig)
    fig = figure;
    set(fig,'numbertitle','off','name',fname, ...
      'userdata',[]);
    handle = figure(fig);
    set(gca,'NextPlot','add');
    [state,optimvalues] = callOnePlotFcn(solverName,func{i},state,options,optimvalues,'init',fcnArgs{i}{:});
    set(handle,'DeleteFcn',{@mouseaction,[],plotNames{i},'remove'});
  end
  set(0,'CurrentFigure',fig);
  [state,optimvalues] = callOnePlotFcn(solverName,func{i},state,options,optimvalues,flag,fcnArgs{i}{:});
  set(gcf,'DeleteFcn',{@mouseaction,[],func{i},'remove'});
end
%-----------------------------------------------------------
%BEFORECLOSE CloseRequestFcn for main figure window
%-----------------------------------------------------------
function beforeClose(obj,event,gadsSolverState)
% Determine the length of stack. If length is one then we don't
% need a question dialog; we simply delete the obj (close the figure)
callStack = dbstack;
if length(callStack) == 1
  delete(obj)
  return;
end
msg = sprintf('%s\n%s','YES will stop the solver (if running) and close the figure.',...
  'NO will cancel this request.');
handle = questdlg(msg,'Close dialog', 'YES','NO','NO');
switch handle
  case 'YES'
    delete(obj)
    setappdata(0,gadsSolverState,'stop');
  case 'NO'
    return;
  otherwise
    return;
end
%-------------------------------------------------------------------------

function [rows, cols] = plotLayout(functions)
% Determine the layout size in the figure
rows  = ceil(sqrt(length(functions)));
cols  = ceil(length(functions)/rows);
% Special rule when we have three functions.
if length(functions) == 3
  rows = 3;
  cols = 1;
end
