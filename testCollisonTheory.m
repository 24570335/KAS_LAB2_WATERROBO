% Initialize BrushBot
clf;
hold on;
robot = BrushBot;
r = UR3e_adjusted;
robot.model.plot3d([0, 0, 0, 0, 0]); % Plot the robot in its initial position
r.model.plot3d([0,0,0,0,0,0]);
robot.model.n;
drawnow();

% Define radii for each link’s ellipsoid
radiiList = [
    0.1, 0.2, 0.1; % Link 1
    0.16, 0.09, 0.09; % Link 2
    0.14, 0.09, 0.09; % Link 3
    0.2, 0.09, 0.1; % Link 4
 %   0.03, 0.03, 0.03  % Link 5
];

% Set axis limits to avoid auto-scaling
axis([-1 1 -1 1 -1 1]);  % Adjust these limits based on robot's size
axis equal;

% Camera settings to control zoom level and view angle
camva(10); % Adjust zoom level

% Initialize transformation matrices and joint configuration
q = [0, 0, 0, 0, 0]; % Initial joint configuration
tr = zeros(4, 4, 5);
tr(:,:,1) = robot.model.base;
L = robot.model.links;

% Calculate transformations for each link
for i = 1:4
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
end
centerPoint = [0,0,0];
% Plot each ellipsoid at each link
for i = 1:4
    radii = radiiList(i, :);
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
    transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))]; %used to make homogenous matrix, be able to transform all points in one go (list)
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
    surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

%{
% Optional: Add teach panel for interactive control
%robot.model.teach;

%NEXT APPLY MESH VERTICES ONTO TABLE AND WALL AND SINK
[Y,Z] = meshgrid(-0.2:0.05:0.2,-0.2:0.05:0.2);
sizeMat = size(Y);
X = repmat(0.2,sizeMat(1),sizeMat(2));
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
cubeAtOrigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePointsNew = cubePoints + repmat([0,1.5,1],size(cubePoints,1),1);
cube_h = plot3(cubePointsNew(:,1),cubePointsNew(:,2),cubePointsNew(:,3),'b.');
axis equal

%NEXT DETECT
for i = 1:robot.model.n
    % Setting the radii and transforms related to each set of ellipsoid links
    radii = radiiList(i, :);
    cubePointsAndOnes = [inv(tr(:,:,i+1)) * [cubePointsNew, ones(size(cubePointsNew, 1), 1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    
    % Calculate algebraic distances and check points inside
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    disp(['2.10: There are ', num2str(size(pointsInside,1)), ' points inside the ', num2str(i), 'th ellipsoid']);
end


function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

%NEXT MERGE THIS CODE W WASH_SCRIPT

    %{
    % Check distance of each obstacle point to the ellipsoid
    distances = GetAlgebraicDist(obstaclePoints, center, radii);
    
    % Determine if any points are within or close to the ellipsoid
    collisionPoints = obstaclePoints(distances < 1);  % Adjust threshold as needed
    
    % If collision detected, print or handle avoidance
    if ~isempty(collisionPoints)
        disp(['Collision detected on link ', num2str(i)]);
        % Handle avoidance here: e.g., stop movement, re-plan path, etc.
        break;  % Stop checking further once a collision is detected
    end
end
    %}
%}


%{
% Clear previous visualizations, if any
try delete(cubeAtOigin_h); end;
try delete(ellipsoidAtOrigin_h); end;
try delete(oneSideOfCube_h); end;

% Initialize BrushBot
brushBot = BrushBot;  

% Plot the robot (make sure the plot3d line is active to show the robot)
%brushBot.model.plot3d([0, 0, 0, 0, 0]);
axis equal;
hold on; % Ensure all plots stay in the same figure
camlight;

% Define radii for each ellipsoid on each link (modify based on link sizes)
radiiList = [
    0.01, 0.01, 0.01; % Link 1
    0.02, 0.01, 0.01; % Link 2
    0.015, 0.01, 0.01; % Link 3
    0.015, 0.01, 0.01; % Link 4
    0.01, 0.05, 0.05 % Link 5
];

% Set up initial joint angles and transformations
q = [0, 0, 0, 0, 0]; % Initial joint configuration
tr = zeros(4, 4, brushBot.model.n + 1);
tr(:,:,1) = brushBot.model.base;  % Base transformation
L = brushBot.model.links; % Get links of the robot

% Calculate transformations for each link
for i = 1:brushBot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0, 0, L(i).d) * transl(L(i).a, 0, 0) * trotx(L(i).alpha);
end

% Define some sample cube points for testing (assuming cube points are defined)
cubePoints = rand(100, 3) - 0.5; % Random points around origin for testing

% Initialize an array to store ellipsoid handles
ellipsoidHandles = gobjects(brushBot.model.n, 1);

% Go through each link and create the ellipsoid around it
for i = 1:brushBot.model.n
    % Generate ellipsoid based on radii
    radii = radiiList(i, :);
    [X, Y, Z] = ellipsoid(0, 0, 0, radii(1), radii(2), radii(3));
    
    % Transform the ellipsoid points to the link frame
    transformedPoints = tr(:,:,i+1) * [X(:), Y(:), Z(:), ones(numel(X), 1)]';
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));

    % Plot the ellipsoid at each link and store the handle
    ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed);
    set(ellipsoidHandles(i), 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Semi-transparent

    % Optionally: Check which points are inside the ellipsoid using algebraic distance
    % cubePointsAndOnes = [inv(tr(:,:,i+1)) * [cubePoints, ones(size(cubePoints, 1), 1)]']';
    % updatedCubePoints = cubePointsAndOnes(:, 1:3);
    % algebraicDist = GetAlgebraicDist(updatedCubePoints, [0, 0, 0], radii);
    % pointsInside = find(algebraicDist < 1);
    % disp(['Link ', num2str(i), ': ', num2str(size(pointsInside, 1)), ' points inside the ellipsoid']);
end

% Set up lighting and final visual adjustments
camlight;
lighting gouraud;
%}







% 2.10
% q = [0,0,0];
% tr = zeros(4,4,robot.n+1);
% tr(:,:,1) = robot.base;
% L = robot.links;
% for i = 1 : robot.n
%     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% end



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