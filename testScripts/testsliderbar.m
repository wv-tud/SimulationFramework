F = figure(2);
set(F,'Position',[100 100 500 600]);
h = uicontrol('style','slider','units','pixel','position',[20 20 450 20],'min',1,'max',200,'sliderstep',[1/199 1/199],'value',1);
addlistener(h,'ContinuousValueChange',@(hObject, event) uArena{1}.agents{round(h.Value)}.plotVelocityComponents());
