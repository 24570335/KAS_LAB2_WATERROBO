% Clearing previous visual depictions:
try delete(cubeAtOigin_h); end
try delete(ellipsoidAtOrigin_h); end
try delete(oneSideOfCube_h); end

% adjust for to ur5 and new brushbot        
robot = BrushBot;  

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [0.1,0.05,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});    
    warning on;
end

%robot.plot3d([0,0,0]);

%axis equal
%camlight
robot.model.plot3d([0,0,0]);
%axis equal
%camlight

robot.model.teach