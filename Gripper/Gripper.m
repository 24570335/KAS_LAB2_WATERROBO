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
<<<<<<< HEAD
            self.model.base = self.model.base.T;% * transl(0.19,1.40,0.99) * trotz(pi/2);
=======
            self.model.base = self.model.base.T * transl(0.17,1.6,0.99) * trotz(-pi/2);
>>>>>>> 6b462852ccfd02498fbbab57d634e596749bb6ba
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
<<<<<<< HEAD
            link(1) = Link('d',-0.1,     'a',0.03,      'alpha',0,'offset',pi,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link([0     .5       0       0    1]);
            link(2).qlim = [0 0.4];
            self.model = SerialLink(link,'name',self.name);            
=======
            % top a = y, d = z, 
            % Link 1: Rotate by 90 degrees to align z-axis with y-axis
            link(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0, ...
                           'qlim', [deg2rad(-360), deg2rad(360)]);
            
            % bottom
            % Link 2: Prismatic joint to move along the new y-axis
            link(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0, ...
                           'qlim', [deg2rad(-360), deg2rad(360)]);
        
            % Combine links into SerialLink model
            self.model = SerialLink([link(1) link(2)], 'name', '2-Link with Y Movement');            
<<<<<<< HEAD
        end  
=======
>>>>>>> 6b462852ccfd02498fbbab57d634e596749bb6ba
        end    
>>>>>>> 1534c06a29f8ef64d6bbee87b0f630012ee3dbf6
    end
end
