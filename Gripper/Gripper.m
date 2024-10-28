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
          
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end
