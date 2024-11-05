classdef BrushTESTBot < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'BrushTESTBot'
    end
   
    
    methods
%% Constructor
        function self = BrushTESTBot(baseTr)
            self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end          
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.089159,    'a',0,      'alpha', pi/2 ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',-0.08,         'a',-0.225,  'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            link(3) = Link('d',0.08,         'a',-0.2,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',-0.08,     'a',-0.2,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0.8,     'a',-0.2,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end