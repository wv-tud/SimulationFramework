classdef figureTable < handle
    %FIGURETABLE writes table to figure
    %   Class that writes a table containing data to a figure
    
    properties
        height      = 0;
        width       = 200;
        rowHeight   = 23;
        position    = [0 700];
        rowI        = 0;
        cellPadding = 10;
        rowText     = {};
        resolution;
        fig_h;
        axes_h;
    end
    
    methods
        function obj = figureTable(posX,posY,resolution,col1,col2)
            obj.position    = [posX posY];
            obj.resolution  = resolution;
            %obj.fig_h   = figure('Position',[0 0 1250 1000]);
            obj.axes_h  = axes('Position',[((0.9*(obj.resolution(1)-200))/obj.resolution(1)+0.06+25/obj.resolution(1)) 0 1 1],'Visible','off');
            hold on;
            obj.axes_h.XLimMode = 'manual';
            obj.axes_h.YLimMode = 'manual';
            obj.axes_h.XLim = [0 obj.resolution(1)];
            obj.axes_h.YLim = [0 obj.resolution(2)];
            obj.build(col1,col2);
        end
        
        function build(obj,col1,col2)
            for i=1:length(col1)
                if isa(col2{i},'double')
                    col2{i} = num2str(col2{i});
                end
                obj.buildRow(col1{i},col2{i});
            end
            set(obj.axes_h,'Visible','off');
            obj.axes_h.XLim = [0 obj.resolution(1)];
            obj.axes_h.YLim = [0 obj.resolution(2)];
            hold off;
            
        end
        
        function buildRow(obj,varargin)
            rowLX       = obj.position(1);
            rowTY       = obj.position(2) + obj.rowHeight*obj.rowI;
            plot([rowLX rowLX+obj.width],[rowTY rowTY],'k');
            plot([rowLX rowLX+obj.width],[rowTY-obj.rowHeight rowTY-obj.rowHeight],'k');
            plot([rowLX rowLX],[rowTY rowTY-obj.rowHeight],'k');
            plot([rowLX+obj.width rowLX+obj.width],[rowTY rowTY-obj.rowHeight],'k');
            plot([rowLX+0.5*obj.width rowLX+0.5*obj.width],[rowTY rowTY-obj.rowHeight],'k');
            obj.rowText{obj.rowI+1}{1} = text(obj.cellPadding+rowLX, rowTY-0.5*obj.rowHeight,varargin{1},'FontSize',12);
            obj.rowText{obj.rowI+1}{2} = text(obj.cellPadding+rowLX+0.5*obj.width, rowTY-0.5*obj.rowHeight,varargin{2},'FontSize',12);
            obj.rowI    = obj.rowI + 1;
            obj.height  = obj.height + obj.rowI * obj.rowHeight;
        end
        
        function updateTable(obj,arr)
            for i=1:length(arr)
                obj.updateRow(i,arr{i})
            end
        end
        
        function updateRow(obj,i,text)
            set(obj.fig_h, 'currentaxes', obj.axes_h);
            if isa(text,'double')
                text = num2str(text);
            end
            obj.rowText{i}{2}.String = text;
        end
    end
    
end

