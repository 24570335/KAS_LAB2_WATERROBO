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
            self.model.base = self.model.base.T;% * transl(0.19,1.40,0.99) * trotz(pi/2);
            self.model.base = self.model.base.T * transl(0.17,1.6,0.99) * trotz(-pi/2);
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
            
            link(1) = Link([0     0       0       0    1]);
            link(1).qlim = [0 0.2];
            link(2) = Link([0     0       0       0    1]);
            link(2).qlim = [0 -0.2];

            %this does not work as it says arrays have incompatible sizes.
            %issue in robotBaseClass - PlotAndColourRobot
            %link(1) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', pi/2, 'qlim', [0, 0.2]);
            %link(2) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-0.2, 0]);
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end
