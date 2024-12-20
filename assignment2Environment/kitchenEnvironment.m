classdef kitchenEnvironment < handle
    %#ok<*NASGU>
    %#ok<*NOPRT>
    %#ok<*TRYNC>
    
    % Calling the functions from down below in my class:
    methods 
        function self = kitchenEnvironment()
            clf
            clc
            input('Press enter to begin')
            self.EnvironmentSafety();
        end
    end 

    methods(Static)
        function EnvironmentSafety()
            hold on;
            % Setting up workspace
            workspace = [-4 3 -2 2 0 5];
            axis(workspace);
    
           


            % Setting up table, person ply + safety fire extinguisher, emergency stop
            PlaceObject('table.ply',[-1,0.9,0.15]);
            %PlaceObject('personFemaleBusiness.ply',[1.9,0.8,0]);
            PlaceObject('fireExtinguisherElevated.ply',[0.9,1.2,0.45]);
            PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [2,1.5,0]);
            PlaceObject('emergencyStopButton.ply', [0.9,0.5,0]);
            
            % Fence placement parallel and rotated:
            PlaceObject('barrier1.5x0.2x1m.ply',[-0.4,-0.4,0]);
            PlaceObject('barrier1.5x0.2x1m.ply',[-1.9,-0.4,0]);

            h8 = PlaceObject('barrier1.5x0.2x1m.ply'); % rotating fence
            verts = get(h8,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [0.5, 0.5, 0];  % Updated y position
            initial_transform = transl(initial_position)*trotz(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(h8,'Vertices',transformedVerts(:,1:3));

            h82 = PlaceObject('barrier1.5x0.2x1m.ply'); % rotating fence
            verts = get(h82,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [0.6, 1.2, 0];  % Updated y position
            initial_transform = transl(initial_position)*trotz(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(h82,'Vertices',transformedVerts(:,1:3));

            h9 = PlaceObject('barrier1.5x0.2x1m.ply'); % rotating fence
            verts = get(h9,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-2.5, 0.5, 0];  % Updated y position
            initial_transform = transl(initial_position)*trotz(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(h9,'Vertices',transformedVerts(:,1:3));

            h92 = PlaceObject('barrier1.5x0.2x1m.ply'); % rotating fence
            verts = get(h92,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-2.6, 1.2, 0];  % Updated y position
            initial_transform = transl(initial_position)*trotz(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(h92,'Vertices',transformedVerts(:,1:3));

            % E-stop wall mounted ply set-up:
            h10 = PlaceObject('emergencyStopWallMounted.ply');
            verts = get(h10,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [2,1.5,0.5];  % Updated y position
            initial_transform = transl(initial_position)*trotx(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(h10,'Vertices',transformedVerts(:,1:3));

            % Setting up sink and applying transformations
            h_sink = PlaceObject('sink.ply');  % Place the sink near the wall (adjust coordinates as needed
            verts = get(h_sink, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-100,170,0.5];  % Updated y position
            rotation_transform = transl(initial_position)*trotx(pi/2)*troty(pi/2);  % Rotate sink upright
            transformedVerts = (rotation_transform * verts_homogeneous')';
            set(h_sink, 'Vertices', transformedVerts(:, 1:3));
            scalingFactor = 0.01;
            scalingMatrix = diag([scalingFactor, scalingFactor, scalingFactor]);
            scaledVerts = (scalingMatrix * transformedVerts(:, 1:3)')';
            set(h_sink, 'Vertices', scaledVerts);

            % Setting up warning light and applying transformations
            warning_light = PlaceObject('warninglight.ply');  % Place above the sink, near the wall (adjust coordinates as needed
            verts = get(warning_light, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-100,190,200];  % Updated x,y,z position
            rotation_transform = transl(initial_position)*trotz(pi/2)*troty(pi/2);  % Rotate warning light upright
            transformedVerts = (rotation_transform * verts_homogeneous')';
            set(warning_light, 'Vertices', transformedVerts(:, 1:3));
            scalingFactor = 0.01;
            scalingMatrix = diag([scalingFactor, scalingFactor, scalingFactor]);
            scaledVerts = (scalingMatrix * transformedVerts(:, 1:3)')';
            set(warning_light, 'Vertices', scaledVerts);

            % Table holds to elevate table to same level as the kitchen
            % sink / other robot:
            table_hold1 = PlaceObject('table_hold.ply');
            verts = get(table_hold1,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-1,1.3,0.15];  % Updated y position
            initial_transform = transl(initial_position)*trotx(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(table_hold1,'Vertices',transformedVerts(:,1:3));

            table_hold2 = PlaceObject('table_hold.ply');
            verts = get(table_hold2,'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-2,1.3,0.15];  % Updated y position
            initial_transform = transl(initial_position)*trotx(pi/2);
            transformedVerts = (initial_transform * verts_homogeneous')';
            set(table_hold2,'Vertices',transformedVerts(:,1:3));

            % Setting up wet_floor_sign and applying transformations
            wet_floor_sign = PlaceObject('wet_floor_sign.ply');  % Place next to barrier where visible (adjust coordinates as needed
            verts = get(wet_floor_sign, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-300,-60,30];  % Updated x,y,z position
            rotation_transform = transl(initial_position); 
            transformedVerts = (rotation_transform * verts_homogeneous')';
            set(wet_floor_sign, 'Vertices', transformedVerts(:, 1:3));
            scalingFactor = 0.01;
            scalingMatrix = diag([scalingFactor, scalingFactor, scalingFactor]);
            scaledVerts = (scalingMatrix * transformedVerts(:, 1:3)')';
            set(wet_floor_sign, 'Vertices', scaledVerts);

            % Setting up general warning and applying transformations
            general_warning = PlaceObject('iso_7010_w001.ply');  % Place above the sink, near the wall (adjust coordinates as needed
            verts = get(general_warning, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [0,398,400];  % Updated x,y,z position
            rotation_transform = transl(initial_position)*trotx(pi/2);  % Rotate general warning upright
            transformedVerts = (rotation_transform * verts_homogeneous')';
            set(general_warning, 'Vertices', transformedVerts(:, 1:3));
            scalingFactor = 0.005;
            scalingMatrix = diag([scalingFactor, scalingFactor, scalingFactor]);
            scaledVerts = (scalingMatrix * transformedVerts(:, 1:3)')';
            set(general_warning, 'Vertices', scaledVerts);
            
            % Setting up battery warning and applying transformations
            general_warning = PlaceObject('iso_7010_w026.ply');  % Place above the sink, near the wall (adjust coordinates as needed
            verts = get(general_warning, 'Vertices');
            verts_homogeneous = [verts, ones(size(verts, 1), 1)];
            initial_position = [-600,398,400];  % Updated x,y,z position
            rotation_transform = transl(initial_position)*trotx(pi/2);  % Rotate battery warning upright
            transformedVerts = (rotation_transform * verts_homogeneous')';
            set(general_warning, 'Vertices', transformedVerts(:, 1:3));
            scalingFactor = 0.005;
            scalingMatrix = diag([scalingFactor, scalingFactor, scalingFactor]);
            scaledVerts = (scalingMatrix * transformedVerts(:, 1:3)')';
            set(general_warning, 'Vertices', scaledVerts);

           

            % Wooden floor image:
            surf([-4,-4;4,4] ...
            ,[-4,4;-4,4] ...
            ,[0.01,0.01;0.01,0.01] ...
            ,'CData',imread('woodFloor.jpeg') ...
            ,'FaceColor','texturemap');    
            
            % Kitchen wall tiles, now perpendicular to the floor
            surf([-4,-4;4,4] ...   % x-coordinates (same)
            ,[2,2;2,2] ...     % y-coordinates, set to 0 to position it along y=0
            ,[0,8;0,8] ...     % z-coordinates to create vertical height
            ,'CData',imread('kitchenTiles.jpeg') ...
            ,'FaceColor','texturemap');
  
            % Label the axes
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            
            % Initialising robot and base:
            hold on;
        end
    end
end
