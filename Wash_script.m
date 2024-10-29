% Initialising robot and base:
hold on
kitchenEnvironment;

%Loading bottle body and cap:
% Setting up BOTTLE BODY and applying transformations
bottleBody = PlaceObject('BottleBody3.ply', [0,0,0]);
vertsBody = get(bottleBody, 'Vertices');
vertsHomogeneousBody = [vertsBody, ones(size(vertsBody, 1), 1)];
initialPositionBody = [-1.1,1.1,.82];  % Updated x,y,z position
rotationTransformBody = transl(initialPositionBody)*trotx(pi/2);  % Rotate 
transformedVertsBody = (rotationTransformBody * vertsHomogeneousBody')';
set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

% Setting up BOTTLE CAP and applying transformations
bottleCap = PlaceObject('BottleCap3.ply', [0,0,0]);  % Place on top of table, on top of BOTTLE BODY (adjust coordinates as needed
vertsCap = get(bottleCap, 'Vertices');
vertsHomogeneousCap = [vertsCap, ones(size(vertsCap, 1), 1)];
initialPositionCap = [-1.1,1.1,1];  % Updated x,y,z position
rotationTransformCap = transl(initialPositionCap)*trotx(pi/2);  % Rotate 
transformedVertsCap = (rotationTransformCap * vertsHomogeneousCap')';
set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));

r = UR3e_adjusted;
% change to original stuff:
waypoints = [71*pi/180,0,0,0,0,0;
             pi,-1/12,1/12,-pi/2,3*pi/2,-pi/2;
             2*pi*7/18,-1/12,1/12,-pi/2,2*pi,0;
             pi/4,0,0,pi/-2,2*pi,0;];

% waypoints used in real demo:
% waypoints = [71*pi/180,-pi/6,0,0,0,0;
%              pi,-0.452,0,0,0,pi/2;
%              2*pi*7/18,-pi/6,1/12,-pi/2,0,pi/2;
%              pi/4,-pi/6,0,pi/-2,0,pi/2;];

br = BrushBot;
waypoints2 = [0,-pi/2,0,0,0;
             0,-109*pi/90,pi*19/90,0,0;
             0,-pi/2,0,0,0;
             0,-pi/2,0,0,0];


grip = Gripper;
% grip.model.base = br.model.fkine(br.model.getpos());
qopen = [0,0.2,0.4];
qclose = [0,0.1,0.2];
rawgrip = rawgripper;

%% Creating ellipsoids on robot
% Initialize array to hold the ellipsoid surface handles for each link
ellipsoidHandles = gobjects(1, 5);  % Pre-allocate for 5 links

% Define radii for each link’s ellipsoid
radiiList = [
    0.1, 0.2, 0.1; % Link 1
    0.16, 0.09, 0.09; % Link 2
    0.14, 0.09, 0.09; % Link 3
    0.2, 0.09, 0.1; % Link 4
    0.03, 0.03, 0.03  % Link 5
];

% Initialize transformation matrices and joint configuration
q0 = [0,0,0,0,0,0];
tr = zeros(4, 4, r.model.n + 1);
tr(:,:,1) = r.model.base;
L = r.model.links;

% Calculate transformations for each link
for i = 1:r.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q0(i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
end

% Plot each ellipsoid at each link
for i = 1:5
    radii = radiiList(i, :);
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
    transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))]; %used to make homogenous matrix, be able to transform all points in one go (list)
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
    ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

for i = 1:5
    try delete(ellipsoidHandles(i)); end;
end

%% Creating mesh vertices to check collisions with
% Create a rectangular wall mesh
[Xw, Zw] = meshgrid(-4:0.05:4, -5:0.05:5); % Different ranges for X and Z for a rectangle
sizeMat = size(Xw);
Yw = repmat(2, sizeMat(1), sizeMat(2)); % Y plane remains constant
% Plot one side of the rectangle as a surface
surf(Xw, Yw, Zw,'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsWall = [Xw(:), Yw(:), Zw(:)];
% Plot the rectangular prism's point cloud         
rect_wall = plot3(rectPointsWall(:,1), rectPointsWall(:,2), rectPointsWall(:,3), 'b.');

% Create a rectangular table mesh
[Xt, Yt] = meshgrid(-2:0.05:0, 0.5:0.05:2); % Different ranges for X and Y for a rectangle
sizeMat = size(Xt);
Zt = repmat(0.83, sizeMat(1), sizeMat(2)); % Z plane remains constant
% Plot one side of the rectangle as a surface
surf(Xt, Yt, Zt,'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsTable = [Xt(:), Yt(:), Zt(:)];
% Plot the rectangular prism's point cloud         
rect_table = plot3(rectPointsTable(:,1), rectPointsTable(:,2), rectPointsTable(:,3), 'g.');

% Create a rectangular alarm mesh
[Xa, Za] = meshgrid(-1.6:0.05:-0.4, 1.9:0.05:2.4); % Different ranges for X and Z for a rectangle
sizeMat = size(Xa);
Ya = repmat(1.85, sizeMat(1), sizeMat(2)); % Y plane remains constant
% Plot one side of the rectangle as a surface
surf(Xa, Ya, Za,'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Combine one surface as a point cloud
rectPointsAlarm = [Xa(:), Ya(:), Za(:)];
% Plot the rectangular prism's point cloud         
rect_alarm = plot3(rectPointsAlarm(:,1), rectPointsAlarm(:,2), rectPointsAlarm(:,3), 'r.');

%%

q0 = [0,0,0,0,0,0];
q0_b = [0,0,0,0,0];
steps = 25;

qmatopen = jtraj(grip.model.getpos(), qopen, steps);
qmatclose = jtraj(qopen,qclose,steps);

% Moving UR3e to bottle
q1 = waypoints(1,:);
qMat = jtraj(q0, q1, steps);
q1b = waypoints2(1,:);
qMatb= jtraj(q0_b,q1b,steps);

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
    for i = 1:r.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(j, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
    end

    % Delete the previous ellipsoids if they exist
    for i = 1:5
        try delete(ellipsoidHandles(i)); end;
    end

    % Update each ellipsoid position to match current UR3 link position
    for i = 1:5
        radii = radiiList(i, :);

        % Extract the center point from the transformation matrix
        centerPoint = tr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
        
        % Apply updated transformation to the ellipsoid
        transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));

        
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');

        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % For Wall Mesh
        rectPointsWall1 = [inv(tr(:,:,i+1)) * [rectPointsWall, ones(size(rectPointsWall, 1), 1)]']';
        rectPointsWall2 = rectPointsWall1(:,1:3);
        algebraicDist = GetAlgebraicDist(rectPointsWall2, [0, 0, 0], radii);
        pointsInside = find(algebraicDist < 1);
        disp(['From Wall there are ', num2str(size(pointsInside,1)), ' points inside the ', num2str(i), 'th ellipsoid']);
     

        % For Table Mesh
        rectPointsTable1 = [inv(tr(:,:,i+1)) * [rectPointsTable, ones(size(rectPointsTable, 1), 1)]']';
        rectPointsTable2 = rectPointsTable1(:,1:3);
        algebraicDist = GetAlgebraicDist(rectPointsTable2, [0, 0, 0], radii);
        pointsInside = find(algebraicDist < 1);
        disp(['From Table there are ', num2str(size(pointsInside,1)), ' points inside the ', num2str(i), 'th ellipsoid']);
        % For Alarm Mesh
        rectPointsAlarm1 = [inv(tr(:,:,i+1)) * [rectPointsAlarm, ones(size(rectPointsAlarm, 1), 1)]']';
        rectPointsAlarm2 = rectPointsAlarm1(:,1:3);
        algebraicDist = GetAlgebraicDist(rectPointsAlarm2, [0, 0, 0], radii);
        pointsInside = find(algebraicDist < 1);
        disp(['From Alarm there are ', num2str(size(pointsInside,1)), ' points inside the ', num2str(i), 'th ellipsoid']);
    end

    
    drawnow()
end


% Moving UR3e to sink
q2 = waypoints(2,:);
qMat = jtraj(q1,q2,steps);
q2b = waypoints2(2,:);
qMatb = jtraj(q1b,q2b,steps);
%qMatg = jtraj(q1b,q2b,steps)

for k=1:steps
    r.model.animate(qMat(k,:)); % Animating the movement to sink
    rawgrip.model.base = r.model.fkine(qMat(k,:));
    rawgrip.model.animate([0,0,0]);
    lastLinkBody = r.model.fkine(qMat(k,:));
    transformedVertsBody = (lastLinkBody.T * vertsHomogeneousBody')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

    lastLinkCap = r.model.fkine(qMat(k,:));
    lastLinkCap = lastLinkCap.T * transl(0, 0.18, 0);
    transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    drawnow;
    br.model.animate(qMatb(k,:));    

    grip.model.base = br.model.fkine(qMatb(k,:));
    grip.model.animate([0,0,0]);
    %grip.model.animate(qMatg(k, :)); % Animating the movement to sink

    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:r.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(k, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
    end

    % Delete the previous ellipsoids if they exist
    for i = 1:5
        try delete(ellipsoidHandles(i)); end;
    end

    % Update each ellipsoid position to match current link position
    for i = 1:5

        radii = radiiList(i, :);
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
        
        % Apply updated transformation to the ellipsoid
        transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end


    drawnow()
end


% Moving UR3e to tipping position
q3 = waypoints(3,:);
qMat = jtraj(q2,q3,steps);
q3b = waypoints2(3,:);
qMatb= jtraj(q2b,q3b,steps);
for l=1:steps
    r.model.animate(qMat(l,:)) % Animating the movement of a tipping position
    rawgrip.model.base = r.model.fkine(qMat(l,:));
    rawgrip.model.animate([0,0,0]);
    lastLinkBody = r.model.fkine(qMat(l,:));
    transformedVertsBody = (lastLinkBody.T * vertsHomogeneousBody')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

    lastLinkCap = r.model.fkine(qMat(l,:));
    lastLinkCap = lastLinkCap.T * transl(0, 0.18, 0);
    transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    drawnow;
    grip.model.base = br.model.fkine(qMatb(l,:));
    grip.model.animate([0,0,0]);
    br.model.animate(qMatb(l,:))

    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:r.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(l, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
    end

    % Delete the previous ellipsoids if they exist
    for i = 1:5
        try delete(ellipsoidHandles(i)); end;
    end

    % Update each ellipsoid position to match current link position
    for i = 1:5
        radii = radiiList(i, :);
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
        
        % Apply updated transformation to the ellipsoid
        transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end


    drawnow()
end

% Moving UR3e to drying station
q4 = waypoints(4,:);
qMat = jtraj(q3,q4,steps);
q4b = waypoints2(4,:);
qMatb= jtraj(q3b,q4b,steps);
for m=1:steps
    r.model.animate(qMat(m,:)) % Animating the movement to dry bottle
    rawgrip.model.base = r.model.fkine(qMat(m,:));
    rawgrip.model.animate([0,0,0]);
    lastLinkBody = r.model.fkine(qMat(m,:));
    transformedVertsBody = (lastLinkBody.T * vertsHomogeneousBody')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

    lastLinkCap = r.model.fkine(qMat(m,:));
    lastLinkCap = lastLinkCap.T * transl(0, 0.18, 0);
    transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    grip.model.base = br.model.fkine(qMatb(m,:));
    grip.model.animate([0,0,0]);
    br.model.animate(qMatb(m,:))
    
    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:r.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(m, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
    end

    % Delete the previous ellipsoids if they exist
    for i = 1:5
        try delete(ellipsoidHandles(i)); end;
    end

    % Update each ellipsoid position to match current link position
    for i = 1:5
        radii = radiiList(i, :);
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));
        
        % Apply updated transformation to the ellipsoid
        transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end

    drawnow()
end


% Function used for collision detection with mesh
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end








%{
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
%}