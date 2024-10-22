classdef BrushBot < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'BrushBot'
    end
   
    
    methods
%% Constructor
        function self = BrushBot(baseTr)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * transl(-1.2,1.6,0.82) * trotz(pi());
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end
%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.2,    'a',0,      'alpha', pi/2 ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',-0.08,         'a',-0.20,  'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            link(3) = Link('d',0.05,         'a',-0.14,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',-0.05,     'a',-0.10,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0,     'a',0,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end