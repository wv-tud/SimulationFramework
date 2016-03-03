res1    = [];
res2    = [];
res3    = [];
aPosX   = unifrnd(-52.5,52.5,200,2000);
aPosY   = unifrnd(-34,34,200,2000);

t1      = tic;
[x,y]   = meshgrid(-52.5:0.5:52.5,-34:0.5:34);
v       = sqrt(x.^2+y.^2);
F       = scatteredInterpolant(reshape(x,[],1),reshape(y,[],1),reshape(v,[],1),'linear');
toc(t1)
for i=1:200/0.1
    for j=1:200
        res1(end+1) = F(aPosX(j,i),aPosY(j,i));
    end
end
toc(t1)

t2 = tic;
c_fun = @(pos) sqrt(pos(1).^2+pos(2).^2);
for t=1:200/0.1
    for j=1:200
        res2(end+1) = c_fun([aPosX(j,i) aPosY(j,i)]);
    end
end
toc(t2)

t3 = tic;
c_fun = @(pos) sqrt(pos(1).^2+pos(2).^2);
for t=1:200/0.1
    for j=1:200
        res3(end+1) = feval(c_fun,[aPosX(j,i) aPosY(j,i)]);
    end
end
toc(t3)
clear t1 t2 t3