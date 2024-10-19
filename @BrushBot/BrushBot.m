classdef BrushBot < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'BrushBot'
    end
   
    
    methods
%% Constructor
        function self = BrushBot(baseTr)
            self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end          
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.0,    'a',0,      'alpha', pi/2 ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',-0.08,         'a',-0.20,  'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            link(3) = Link('d',0.05,         'a',-0.14,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',-0.05,     'a',-0.10,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0.00,     'a',0,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end