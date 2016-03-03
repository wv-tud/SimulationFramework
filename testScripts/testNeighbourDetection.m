close all hidden;
clear uArena;
names                   = {'testNeighbourDetection'};
uArena{length(names)}   = {};
for i=1:length(names)  
    uArena{i}               = Arena(names{i});
    uArena{i}.T             = 0.1;
    uArena{i}.dt            = 0.1;
    uArena{i}.nAgents       = 1;
    uArena{i}.size          = [0.01 0.01];
    uArena{i}.print         = 0;
    uArena{i}.save          = 0;
    uArena{i}.p_fov         = 1;
    uArena{i}.p_head        = 0;
    uArena{i}.p_label       = 0;
    uArena{i}.init          = 'random';
    uArena{i}.p_mov_axe     = 1;
    uArena{i}.p_axe_lim     = [15 15];
    uArena{i}.showFig       = 1; % Show     = faster
    %uArena{i}.c_fun         = @(t) [15*sin(2*pi()/720*t) 15*cos(2*pi()/720*t) 10]; 
    %uArena{i}.agent_conf    = struct('n_xyz',n_xyz{i});
    %uArena{i}.agent_conf    = struct('g_fun',@(varargin) varargin{2}./norm(varargin{2}).*[-1 -1 0].*(-1/norm(varargin{2})^2 + varargin{3}));
    %uArena{i}.agent_conf    = struct('g_fun',@(varargin) 5*varargin{3}.*[1 1 0].*varargin{2}/norm(varargin{2}).*[cos(1/5*2*pi()*norm(varargin{2}))+sin(1/5*2*pi()*norm(varargin{2})) cos(1/5*2*pi()*norm(varargin{2}))+sin(1/5*2*pi()*norm(varargin{2})) 0]);
    fprintf(strcat([names{i} ': Initialised arena and agents\n'])); 
    simT = tic;
    uArena{i}.Simulate();
    t = toc(simT);
    fprintf(strcat([names{i} ': Simulation took ' uArena{i}.sec2time(round(t)) 's\n']));
end
% for i=1:length(uArena{1}.agents)
%     uArena{1}.agents{i}.plotVelocityComponents();
% end
for j=1:length(names)
    if uArena{j}.save == 1
        tmp = uArena{j};
        save(strcat(names{j},'.mat'),'tmp');
        clear tmp;
    end
end

%% Start adding random points
lim         = 7.5;
n           = 1000;
init        = 0;

if init==1
    pointPos    = [unifrnd(-lim,lim,n,2) 10*ones(n,1)];
else
    n           = round(sqrt(n))^2;
    [xP,yP]     = meshgrid(linspace(-lim,lim,sqrt(n)),linspace(-lim,lim,sqrt(n)));
    pointPos    = [reshape(xP,[],1) reshape(yP,[],1) 10*ones(n,1)];
end

pointHead   = zeros(n,2);
for i=1:n
    uArena{1}.agents{uArena{1}.nAgents+i}.pos       = pointPos(i,:);
    uArena{1}.agents{uArena{1}.nAgents+i}.heading   = pointHead(i,:);
end
nAgents             = uArena{1}.nAgents;
uArena{1}.nAgents   = nAgents + n;
[neighbours,dAbs,angles] = uArena{1}.detectNeighbours(1);
viewPoints          = find(neighbours(1,:)>0);
figure(uArena{1}.fig);
hold on;
for i=(1+nAgents):(n+nAgents)
    if sum(viewPoints==i)>0
        color = 'red';
    else
        color = 'blue';
    end
    plot3(pointPos(i-nAgents,1),pointPos(i-nAgents,2),pointPos(i-nAgents,3)-10,'x','Color',color);
    %text(pointPos(i-nAgents,1),pointPos(i-nAgents,2),num2str(i-nAgents));
end

th  = -0.5*uArena{1}.agents{1}.cam_fov:2*pi()/360:0.5*uArena{1}.agents{1}.cam_fov;
r   = uArena{1}.agents{1}.cam_range; 

x   = r*cos(th);
y   = r*sin(th);

x_fov = 0:0.05:uArena{1}.agents{1}.cam_range;
y_fov = zeros(size(x_fov));

aPos = uArena{1}.agents{1}.pos(1,:);

roll = [-90/365*2*pi() -45/365*2*pi() 0 45/365*2*pi()];
for i = 1:length(roll)
    rollMat     = uArena{1}.rotMat(roll(i),'x');
    headMat     = uArena{1}.rotMat(-uArena{1}.agents{1}.heading(1,1)-uArena{1}.agents{1}.cam_dir(1),'z');
    pitchMat    = uArena{1}.rotMat(uArena{1}.agents{1}.cam_dir(2),'y');
    
    fovMat1l    = uArena{1}.rotMat(-0.5*uArena{1}.agents{1}.cam_fov,'z');
    fovMat1r    = uArena{1}.rotMat(0.5*uArena{1}.agents{1}.cam_fov,'z');
    tmp         = [x' y' zeros(length(x),1)]*rollMat*pitchMat*headMat;
    tmp_fov     = [x_fov' y_fov' zeros(length(x_fov),1)]*fovMat1l*rollMat*pitchMat*headMat;
    tmp_fov2    = [x_fov' y_fov' zeros(length(x_fov),1)]*fovMat1r*rollMat*pitchMat*headMat;
    plot3(tmp(:,1),tmp(:,2),tmp(:,3),'-.');
    plot3(tmp_fov(:,1),tmp_fov(:,2),tmp_fov(:,3));
    plot3(tmp_fov2(:,1),tmp_fov2(:,2),tmp_fov2(:,3));
end
hold off;
