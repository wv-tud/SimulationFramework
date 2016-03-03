close all;
x = 0:0.01:pi();

k       = 7;
angle   = 0.5*pi();

dt      = 0.1;
v_max   = 2;
th_max  = 200/360*2*pi();

collision_range = 0.3;
fov     = 175/360*2*pi();

angle_v     = asin((v_max*dt)/(1.5+collision_range));

angle_v = (0.5*fov-dt*th_max)/2;

speed_limit = dt*v_max*(1-1./(1+exp(-14*(abs(x)-(0.5*fov-angle_v)))));

figure();
hold all;
plot(x/(2*pi())*365,speed_limit)
plot([rad2deg(0.5*fov) rad2deg(0.5*fov)],[0 1]);
plot([rad2deg(dt*th_max) rad2deg(dt*th_max)],[0 1]);
hold off;
%angle2 = sin((v_max*dt)/(v_max*dt))

