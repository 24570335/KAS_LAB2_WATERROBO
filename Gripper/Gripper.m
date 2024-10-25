classdef Gripper < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'Gripper'
    end
    
    methods
        %% Constructor
        function self = Gripper(baseTr)
            % Prompt the user to select the folder containing the .ply files
            folderPath = uigetdir;
            if folderPath ~= 0
                plyFilesFolder = folderPath;
                % Use the selected folder path for the ply files
                self.plyFileNameStem = fullfile(plyFilesFolder, 'Gripper');
            else
                error('No folder selected for PLY files.');
            end

            self.CreateModel();
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0,    'a',0,      'alpha', pi/2 ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',-0.08,         'a',-0.22,  'alpha',0,'offset',0,'qlim',[deg2rad(-270),deg2rad(270)]);
            link(3) = Link('d',0.05,         'a',-0.32,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',-0.05,     'a',-0.28,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0,     'a',0.03,      'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end
