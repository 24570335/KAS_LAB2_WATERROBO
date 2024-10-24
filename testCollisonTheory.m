% Clearing previous visual depictions:
try delete(cubeAtOigin_h); end
try delete(ellipsoidAtOrigin_h); end
try delete(oneSideOfCube_h); end

% adjust for to ur5 and new brushbot        
robot = BrushBot;  

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [0.1,0.05,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
for i = 1:5
    robot.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});    
    warning on;
end

robot.model.plot3d([0,0,0]);
camlight
robot.model.teach


% 2.10
q = [0,0,0];
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end



% From LABS
%{

% 2.2
ellipsoidAtOrigin_h = surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);
clf;
centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
view(3);
hold on;


% 2.1
clf;
centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
view(3);
hold on;

% 2.2
ellipsoidAtOrigin_h = surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);

% 2.3
% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

% 2.4
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
disp(['2.4: There are ', num2str(size(pointsInside,1)),' points inside']);

% 2.5
centerPoint = [1,1,1];
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
disp(['2.5: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.6
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);          
pointsInside = find(algebraicDist < 1);
disp(['2.6: There are now ', num2str(size(pointsInside,1)),' points inside']);


% 2.7
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)*trotx(pi/4)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
disp(['2.7: There are now ', num2str(size(pointsInside,1)),' points inside']);
pause(1);

% 2.8
try delete(cubeAtOigin_h); end;
try delete(ellipsoidAtOrigin_h); end;
try delete(oneSideOfCube_h); end;

L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
robot = SerialLink([L1 L2 L3],'name','myRobot');  

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [1,0.5,0.5];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

robot.plot3d([0,0,0]);
axis equal
camlight
% robot.teach
% keyboard

% 2.9
q = [0,0,0];

% UPDATE: fkine function now returns an SE3 object.
% To obtain the Transform Matrix, access the
% variable in the object 'T' with '.T'.
tr = robot.fkine(q).T;
cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
disp(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.10
q = [0,0,0];
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each ellipsoid
for i = 1: size(tr,3)
    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    disp(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end

%}