classdef Gripper < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'Gripper'
    end
    
    methods
        %% Constructor
        function self = Gripper(baseTr)            
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
            self.model.base = self.model.base.T * transl(0.19,1.40,0.99) * trotz(pi/2);
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',-0.1,     'a',0.03,      'alpha',0,'offset',pi,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',-0.1,     'a',0.03,     'alpha',0,'offset',-pi,'qlim',[deg2rad(-270),deg2rad(270)]);
            link(3) = Link('d',0.03,     'a',0.03,      'alpha',0,'offset',pi/2,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',0.03,     'a',-0.03,     'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
          
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end
