close all; 
v_max = 4;
swarm_size = 5;

tmpx    = -52.5:0.3:52.5;
tmpy    = -34:0.3:34;

[x,y]   = meshgrid(tmpx,tmpy);
x       = reshape(x,[],1);
y       = reshape(y,[],1);
Rij     = [x y 0*ones(size(x))];

%g = @(varargin) (varargin{2}.^2).*[1 1 0];
%g = @(varargin) (varargin{2}/norm(varargin{2}).*[varargin{3}(1:2) 0]);

%g = @(varargin) varargin{3}.*[1 1 0].*varargin{2}/norm(varargin{2}).*[cos(1/varargin{4}(1)*2*pi()*norm(varargin{2}))+sin(1/varargin{4}(1)*2*pi()*norm(varargin{2})) cos(1/varargin{4}(1)*2*pi()*norm(varargin{2}))+sin(1/varargin{4}(1)*2*pi()*norm(varargin{2})) 0];
%g = @(varargin) 0.2*(-1/4*varargin{2} + 0.2*varargin{3}.*[1 1 0].*varargin{2}/norm(varargin{2}).*[cos(1/5*2*pi()*norm(varargin{2}))+sin(1/5*2*pi()*norm(varargin{2})) cos(1/5*2*pi()*norm(varargin{2}))+sin(1/5*2*pi()*norm(varargin{2})) 0]);
%g = @(varargin) [varargin{2}(1) varargin{2}(2) 0];
g = @(varargin) ...       % Pinciroli global (default=cirular parabolar) varargin{2} = q
            varargin{2}./norm(varargin{2}).*...
            [-0.011*norm(varargin{2})^2 ... 
            -0.011*norm(varargin{2})^2 -0.5]*[cos((1/(1+0.05*norm(varargin{2})))*0.5*pi()) -sin((1/(1+0.05*norm(varargin{2})))*0.5*pi()) 0; sin((1/(1+0.05*norm(varargin{2})))*0.5*pi()) cos((1/(1+0.05*norm(varargin{2})))*0.5*pi()) 0; 0 0 1];
%g = @(varargin) varargin{2}./norm(varargin{2}).*[-1 -1 0].*varargin{3}.*(1-1/(0.01*norm(varargin{2})+1)^4);

g_i = zeros(length(Rij),3);
for i=1:length(Rij)
    g_i(i,:) = feval(g,[0 0 0],Rij(i,:),v_max.*[1 1 0],swarm_size.*[1 1 0]);
end
%surf(x,y,g_i(1,:)',g_i(2,:)',g_i(3,:)');
%axis([min(x) max(x) -1 1.1*varargin{3}]);
f = figure(1);
set(f,'Position',[0 0 1470 1000]);
hold on;
vNorm = reshape(sqrt(g_i(:,1).^2+g_i(:,2).^2+g_i(:,3).^2),length(tmpy),length(tmpx));
surf(tmpx,tmpy,vNorm,'EdgeColor','none','LineStyle','none');
%figure(2);

resfac = 2;
nor = sum(sqrt(g_i(:,1).^2 + g_i(:,2).^2 + g_i(:,3).^2),2);
u = reshape(1./nor.*g_i(:,1),length(tmpy),length(tmpx));
v = reshape(1./nor.*g_i(:,2),length(tmpy),length(tmpx));
w = reshape(1./nor.*g_i(:,3),length(tmpy),length(tmpx));



quiver3(tmpx(1:resfac:end),tmpy(1:resfac:end),vNorm(1:resfac:end,1:resfac:end),1*u(1:resfac:end,1:resfac:end),1*v(1:resfac:end,1:resfac:end),1*w(1:resfac:end,1:resfac:end),0,'Color','k');
hold off;
axis equal tight;
colorbar();