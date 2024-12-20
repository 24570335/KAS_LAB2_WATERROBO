%% Main script:
% Initialising robot and base:
hold on
kitchenEnvironment;

%% Loading bottle body and cap:
% Setting up BOTTLE BODY and applying transformations
bottleBody = initialiseBottlePart('BottleBody3.ply', [-1.1, 1.1, 0.82]);
bottleCap = initialiseBottlePart('BottleCap3.ply', [-1.1, 1.1, 1]);

%% Initialising robots and their movement waypoints / grippers
r = UR3e_adjusted;
waypointsUR3 = setupWaypointsUR3();
br = BrushBot;
waypointsBR = setupWaypointsBR();
grip = Gripper;
qopen = [0, 0.2, 0.4];
qclose = [0, 0.1, 0.2];
rawgrip = rawgripper;

%% Creating ellipsoids on robot UR3
% Initialising array to hold the ellipsoid surface handles for each link
radiiList = setupRadiiList();
ellipsoidHandles = gobjects(1, 5);  % Pre-allocate for 5 links
% Initialize transformation matrices and joint configuration
q0 = [0,0,0,0,0,0];
tr = zeros(4, 4, r.model.n + 1);
tr(:,:,1) = r.model.base;
L = r.model.links;
% Plot each ellipsoid at each link
for i = 1:5
    tr(:,:,i+1) = tr(:,:,i) * trotz(q0(i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
    radii = radiiList(i, :);
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
    transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))]; %used to make homogenous matrix, be able to transform all points in one go (list)
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
    ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0, 'EdgeColor', 'none', 'FaceColor','none');
end
for i = 1:5
    try delete(ellipsoidHandles(i)); end;
end

%% Creating ellipsoids on robot BrushBot
% Initialising array to hold the ellipsoid surface handles for each link
radiiListBr = setupRadiiListBr();
ellipsoidHandlesBr = gobjects(1, 4);  % Pre-allocate for 4 links
% Initialize transformation matrices and joint configuration
q0 = [0,0,0,0,0,0];
trBr = zeros(4, 4, 5);
trBr(:,:,1) = br.model.base;
LBr = br.model.links;
% Plot each ellipsoid at each link
for i = 1:4
    trBr(:,:,i+1) = trBr(:,:,i) * trotz(q0(i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
    radiiBr = radiiListBr(i, :);
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
    transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))]; %used to make homogenous matrix, be able to transform all points in one go (list)
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
    ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0, 'EdgeColor', 'none', 'FaceColor','none');
end
for i = 1:4
    try delete(ellipsoidHandlesBr(i)); end;
end

%% Creating mesh vertices to check collisions with
% Create a rectangular wall mesh
[Xw, Zw] = meshgrid(-4:0.05:4, -5:0.05:5); % Different ranges for X and Z for a rectangle
sizeMat = size(Xw);
Yw = repmat(2, sizeMat(1), sizeMat(2)); % Y plane remains constant
% Plot one side of the rectangle as a surface
surf(Xw, Yw, Zw,'FaceAlpha', 0, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsWall = [Xw(:), Yw(:), Zw(:)];
% Plot the rectangular prism's point cloud         
rect_wall = plot3(rectPointsWall(:,1), rectPointsWall(:,2), rectPointsWall(:,3), 'b.', 'Visible', 'off');

% Create a rectangular table mesh
[Xt, Yt] = meshgrid(-2:0.05:0, 0.5:0.05:2); % Different ranges for X and Y for a rectangle
sizeMat = size(Xt);
Zt = repmat(0.83, sizeMat(1), sizeMat(2)); % Z plane remains constant
% Plot one side of the rectangle as a surface
surf(Xt, Yt, Zt,'FaceAlpha', 0, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsTable = [Xt(:), Yt(:), Zt(:)];
% Plot the rectangular prism's point cloud         
rect_table = plot3(rectPointsTable(:,1), rectPointsTable(:,2), rectPointsTable(:,3), 'g.', 'Visible', 'off');

% Create a rectangular alarm mesh
[Xa, Za] = meshgrid(-1.6:0.05:-0.4, 1.9:0.05:2.4); % Different ranges for X and Z for a rectangle
sizeMat = size(Xa);
Ya = repmat(1.85, sizeMat(1), sizeMat(2)); % Y plane remains constant
% Plot one side of the rectangle as a surface
surf(Xa, Ya, Za,'FaceAlpha', 0, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsAlarm = [Xa(:), Ya(:), Za(:)];
% Plot the rectangular prism's point cloud         
rect_alarm = plot3(rectPointsAlarm(:,1), rectPointsAlarm(:,2), rectPointsAlarm(:,3), 'r.', 'Visible', 'off');

%% Final intialisations for trajectory
q0 = [0,0,0,0,0,0];
q0_b = [0,0,0,0,0];
steps = 25;
qmatopen = jtraj(grip.model.getpos(), qopen, steps);
qmatclose = jtraj(qopen,qclose,steps);
% Collision detection boolean
collisionDetectedB = false;
collisionDetectedBr = false;

%% Moving UR3e to bottle WITH RMRC??
q1 = waypointsUR3(1,:);
qMat = jtraj(q0, q1, steps);
q1b = waypointsBR(1,:);
qMatb= jtraj(q0_b,q1b,steps);
deltaT = 0.05; % RMRC time step
for j=1:steps
    r.model.animate(qMat(j, :)) % Animating the movement to bottle
    rawgrip.model.base = r.model.fkine(qMat(j,:));
    rawgrip.model.animate([0,0,0]);
    br.model.animate(qMatb(j,:))
    grip.model.base = br.model.fkine(qMatb(j,:));
    grip.model.animate([0,0,0]);
    % grip.model.animate(qmatopen(j,:))

    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:5
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(j, i)) * transl(0, 0, L(i).d) * transl(L(i).a / 1.3, 0, 0) * trotx(L(i).alpha);
        [radiiBr, centerPoint, X_transformed, Y_transformed, Z_transformed] = updateLinkTransformation(i, qMat, j, tr, L, radiiList);
        try delete(ellipsoidHandles(i)); end;
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'b');

        % Call the checkCollisions function
        collisionDetectedB = checkCollisions(rectPointsWall, rectPointsTable, rectPointsAlarm, centerPoint, radiiBr, i, tr);
        % If collision detected, exit
        if collisionDetectedB
            disp('Collision detected.');
            return;
        end
        
    end
    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(j, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a / 1.3, 0, 0) * trotx(LBr(i).alpha);
        [radiiBr, centerPoint, X_transformed, Y_transformed, Z_transformed] = updateLinkTransformation(i, qMatb, j, trBr, LBr, radiiListBr);
        try delete(ellipsoidHandlesBr(i)); end;
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'b');
        
        % Call the checkCollisions function
        collisionDetectedBr = checkCollisions(rectPointsWall, rectPointsTable, rectPointsAlarm, centerPoint, radiiBr, i, trBr);
        % If collision detected, exit
        if collisionDetectedBr
            disp('Collision detected.');
            return;
        end
    end
    drawnow()
end


function [radiiBr, centerPoint, X_transformed, Y_transformed, Z_transformed] = updateLinkTransformation(index, traject, trajStep, tr, L, radiiList)
    % Define ellipsoid parameters for the current link
    radiiBr = radiiList(index, :);
    centerPoint = tr(1:3, 4, index+1)'; % Extract the center point
    
    % Generate ellipsoid data
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
    transformedPoints = tr(:,:,index+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
    
    % Reshape transformed points for plotting
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
end

function collisionDetected = checkCollisions(rectPointsWall, rectPointsTable, rectPointsAlarm, centerPoint, radii, linkIndex, tr)
    % Perform collision checks with each object
    pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radii, 'Wall', linkIndex, tr);
    pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radii, 'Table', linkIndex, tr);
    pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radii, 'Alarm', linkIndex, tr);

    % Check thresholds for each object to determine collision
    if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
        collisionDetected = true;
    else
        collisionDetected = false;
    end
end

%% EXTRA FUNCTIONS:
% Function used for collision detection with mesh
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

% Function to initialise each bottle part and its vertices
function part = initialiseBottlePart(filename, position)
    part = PlaceObject(filename, [0, 0, 0]);
    verts = get(part, 'Vertices');
    vertsHomogeneous = [verts, ones(size(verts, 1), 1)];
    transform = transl(position) * trotx(pi/2);
    transformedVerts = (transform * vertsHomogeneous')';
    set(part, 'Vertices', transformedVerts(:, 1:3));
end

% Function to setup waypointsUR3 for UR3
function waypointsUR3 = setupWaypointsUR3()
    waypointsUR3 = [71 * pi/180, 0, 0, 0, 0, 0;
                 pi, -1/12, 1/12, -pi/2, 3*pi/2, -pi/2;
                 2 * pi * 7/18, -1/12, 1/12, -pi/2, 2*pi, 0;
                 pi/4, 0, 0, pi/-2, 2*pi, 0];
end

% Function to setup waypoints for BrushBot
function waypointsBR = setupWaypointsBR()
    waypointsBR = [0, -pi/2, 0, 0, 0;
                  0, -109*pi/90, pi*19/90, 0, 0;
                  0, -pi/2, 0, 0, 0;
                  0, -pi/2, 0, 0, 0];
end

% Function setting up radii list for each link to create an ellipsoid
function radiiList = setupRadiiList()
    radiiList = [
        0.1, 0.2, 0.1;   % Link 1
        0.16, 0.09, 0.09; % Link 2
        0.14, 0.09, 0.09; % Link 3
        0.2, 0.09, 0.1;   % Link 4
        0.03, 0.03, 0.03  % Link 5
    ];
end

function radiiListBr = setupRadiiListBr()
    radiiListBr = [
        0.1, 0.2, 0.1;   % Link 1
        0.16, 0.09, 0.09; % Link 2
        0.14, 0.09, 0.09; % Link 3
        0.2, 0.09, 0.1;   % Link 4
    ];
end

% Function to create a rectangular mesh and point cloud
function points = createMesh(xRange, yRange, zRange, color)
    [X, Z] = meshgrid(xRange(1):0.05:xRange(2), zRange(1):0.05:zRange(2));
    sizeMat = size(X);
    Y = repmat(yRange(1), sizeMat(1), sizeMat(2));
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    points = [X(:), Y(:), Z(:)];
    plot3(points(:,1), points(:,2), points(:,3), color);
end

% Function for collision checking and reporting
function pointsInsideCount = collisionCheck(meshPoints, centerPoint, radii, objectName, linkIndex, tr)
    homogeneousPoints = [meshPoints, ones(size(meshPoints, 1), 1)];
    transformedPoints1 = (inv(tr(:,:,linkIndex+1)) * homogeneousPoints')';
    transformedPoints = transformedPoints1(:,1:3);
    algebraicDist = GetAlgebraicDist(transformedPoints, [0,0,0], radii);
    pointsInside = find(algebraicDist < 1);
    pointsInsideCount = numel(pointsInside);
    %disp(['From ', objectName, ' there are ', num2str(numel(pointsInside)), ' points inside the ', num2str(linkIndex), 'th ellipsoid']);
end
