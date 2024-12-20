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
    ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
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
    ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
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

%% Hardware eStop initialisation (more info in relevant script)
% delete(serialportfind);
% port_name = '/dev/cu.usbserial-14330';  % Adjust this if the port name is different
% baud_rate = 9600;  % Match the baud rate with the Arduino code
% serialObj = serialport(port_name, baud_rate);
% configureTerminator(serialObj,"CR/LF");
% flush(serialObj);
% serialObj.UserData = struct("Data",[],"Count",1);
% configureCallback(serialObj, "terminator", @(src, event) waitForButtonPress(src));

%% Code for button
%{
delete(serialportfind);
port_name = '/dev/cu.usbserial-14320';  % Adjust this if the port name is different
baud_rate = 9600;  % Match the baud rate with the Arduino code
serialObj = serialport(port_name, baud_rate);
configureTerminator(serialObj, "CR/LF");
flush(serialObj);
serialObj.UserData = struct("Data", [], "Count", 1, "hardwareEstopPress", false);
configureCallback(serialObj, "terminator", @(src, event) checkForEStop(src));
%}

%% Final intialisations for trajectory
q0 = [0,0,0,0,0,0];
q0_b = [0,0,0,0,0];
steps = 25;
qmatopen = jtraj(grip.model.getpos(), qopen, steps);
qmatclose = jtraj(qopen,qclose,steps);
hardwareEstopPress = false;
% Collision detection boolean
collisionDetectedB = false;
collisionDetectedBr = false;
%serialObj.UserData.hardwareEstopPress = false;

%% Moving UR3e to bottle
q1 = waypointsUR3(1,:);
qMat = jtraj(q0, q1, steps);
q1b = waypointsBR(1,:);
qMatb= jtraj(q0_b,q1b,steps);
for j=1:steps
    %checkForEStop(serialObj);
    % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end

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
        % Recalculate transformations for each link in UR3 for each step
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(j, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandles(i)); end;
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
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radii, 'Wall', i, tr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radii, 'Table', i, tr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radii, 'Alarm', i, tr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedB = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end


    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    totalPointsInsideBr = 0;  % Initialise the counter for each step
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(j, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end
    drawnow()
end

% Moving UR3e to sink
q2 = waypointsUR3(2,:);
qMat = jtraj(q1,q2,steps);
q2b = waypointsBR(2,:);
qMatb = jtraj(q1b,q2b,steps);
%qMatg = jtraj(q1b,q2b,steps)

for k=1:steps
    if hardwareEstopPress == true
        disp("E-Stop has been pressed.");
        return;
    end

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
    for i = 1:5
        % Recalculate transformations for each link in UR3 for each step
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(k, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandles(i)); end;
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
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radii, 'Wall', i, tr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radii, 'Table', i, tr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radii, 'Alarm', i, tr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedB = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(k, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end

    drawnow()
end
qspin0= [0,0,0];
qspin1= [4*pi/3,0,0];
longsteps = 100;
qmatspin = jtraj(qspin0,qspin1, longsteps);

for n = 1:longsteps
    % checkForEStop(serialObj);
    % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);
    grip.model.animate(qmatspin(n,:)); %remove cap
    drawnow
end

qMatb= jtraj(q2b,q3b,longsteps);

for o = 1:longsteps
    checkForEStop(serialObj);
    % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);
    grip.model.base = br.model.fkine(qMatb(o,:)); %put cap away
    grip.model.animate([4*pi/3,0,0]);
    lastLinkCap = br.model.fkine(qMatb(o,:));
    lastLinkCap = lastLinkCap.T * transl(0,0,0.1);
    transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    br.model.animate(qMatb(o,:))

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(o, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end
    drawnow
end

qMatb= jtraj(q3b,q0_b,longsteps);

for s = 1:longsteps
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end

    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);
    grip.model.base = br.model.fkine(qMatb(s,:)); %put cap away2
    grip.model.animate([4*pi/3,0,0]);
    lastLinkCap = br.model.fkine(qMatb(s,:));
    lastLinkCap = lastLinkCap.T * transl(0,0,0.1);
    transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));

    br.model.animate(qMatb(s,:))

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(s, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end
    drawnow
end

qMatb= jtraj(q0_b,q1b,longsteps);

for t = 1:longsteps
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);
    grip.model.base = br.model.fkine(qMatb(t,:)); %waypoint to wash
    grip.model.animate([4*pi/3,0,0]);
    br.model.animate(qMatb(t,:))

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(t, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end
    drawnow
end

q2b = waypointsBR(2,:);
qMatb = jtraj(q1b,q2b,longsteps);
for p = 1:longsteps %return to wash
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end

    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);

    grip.model.base = br.model.fkine(qMatb(p,:)); 
    grip.model.animate([4*pi/3,0,0]);
    br.model.animate(qMatb(p,:))
    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(p, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end
    drawnow
end


for q = 1:longsteps %clean
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(q2); % stay
    rawgrip.model.base = r.model.fkine(q2);
    rawgrip.model.animate([0,0,0]);
    grip.model.animate(qmatspin(q,:)); %clean
    drawnow
end

% Moving UR3e to tipping position
steps = 25;
q3 = waypointsUR3(3,:);
qMat = jtraj(q2,q3,steps);
q3b = waypointsBR(3,:);
qMatb= jtraj(q2b,q3b,steps);

for l = 1:steps
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(qMat(l,:)); % Animating the movement of a tipping position
    rawgrip.model.base = r.model.fkine(qMat(l,:));
    rawgrip.model.animate([0,0,0]);
    lastLinkBody = r.model.fkine(qMat(l,:));
    transformedVertsBody = (lastLinkBody.T * vertsHomogeneousBody')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

    % lastLinkCap = r.model.fkine(qMat(l,:));
    % lastLinkCap = lastLinkCap.T * transl(0, 0.18, 0);
    % transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    % set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    drawnow;
    grip.model.base = br.model.fkine(qMatb(l,:));
    grip.model.animate([0,0,0]);
    br.model.animate(qMatb(l,:))

    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:5
        % Recalculate transformations for each link in UR3 for each step
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(l, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandles(i)); end;
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
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radii, 'Wall', i, tr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radii, 'Table', i, tr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radii, 'Alarm', i, tr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedB = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(l, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end


    drawnow()
end

% Moving UR3e to drying station
q4 = waypointsUR3(4,:);
qMat = jtraj(q3,q4,steps);
q4b = waypointsBR(4,:);
qMatb= jtraj(q3b,q4b,steps);
for m=1:steps
    % checkForEStop(serialObj);
    % % Check if e-stop has been pressed
    % if serialObj.UserData.hardwareEstopPress
    %     disp("E-Stop has been pressed.");
    %     return;
    % end
    r.model.animate(qMat(m,:)) % Animating the movement to dry bottle
    rawgrip.model.base = r.model.fkine(qMat(m,:));
    rawgrip.model.animate([0,0,0]);
    lastLinkBody = r.model.fkine(qMat(m,:));
    transformedVertsBody = (lastLinkBody.T * vertsHomogeneousBody')'; % Multiplying new transform by homogenous vertices matrix
    set(bottleBody, 'Vertices', transformedVertsBody(:, 1:3));

    % lastLinkCap = r.model.fkine(qMat(m,:));
    % lastLinkCap = lastLinkCap.T * transl(0, 0.18, 0);
    % transformedVertsCap = (lastLinkCap * vertsHomogeneousCap')'; % Multiplying new transform by homogenous vertices matrix
    % set(bottleCap, 'Vertices', transformedVertsCap(:, 1:3));
    grip.model.base = br.model.fkine(qMatb(m,:));
    grip.model.animate([0,0,0]);
    br.model.animate(qMatb(m,:))
    
    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:5
        % Recalculate transformations for each link in UR3 for each step
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(m, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandles(i)); end;
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
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radii, 'Wall', i, tr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radii, 'Table', i, tr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radii, 'Alarm', i, tr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedB = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end
    end

    % Recalculate transformations for each link in BrushBot for each step
    trBr(:,:,1) = br.model.base;
    for i = 1:4
        % Recalculate transformations for each link in UR3 for each step
        trBr(:,:,i+1) = trBr(:,:,i) * trotz(qMatb(m, i)) * transl(0, 0, LBr(i).d) * transl(LBr(i).a/1.3, 0, 0) * trotx(LBr(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandlesBr(i)); end;
        radiiBr = radiiListBr(i, :);
        % Extract the center point from the transformation matrix
        centerPoint = trBr(1:3, 4, i+1)'; % Extract translation components as centerPoint
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radiiBr(1), radiiBr(2), radiiBr(3));
        % Apply updated transformation to the ellipsoid
        transformedPoints = trBr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));
        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandlesBr(i) = surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % Using GetAlgebraicDist find how many points are inside each ellipsoid
        % Use `collisionCheck` for each object mesh
        pointsInsideCountWall = collisionCheck(rectPointsWall, centerPoint, radiiBr, 'Wall', i, trBr);
        pointsInsideCountTable = collisionCheck(rectPointsTable, centerPoint, radiiBr, 'Table', i, trBr);
        pointsInsideCountAlarm = collisionCheck(rectPointsAlarm, centerPoint, radiiBr, 'Alarm', i, trBr);
        
        % Check thresholds for each object
        if pointsInsideCountWall > 1 || pointsInsideCountTable > 6 || pointsInsideCountAlarm > 1
            collisionDetectedBr = true;
            disp('Collision detected.');
            return; % Exit the inner loop
        end 
    end
    drawnow()
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
    transformedPoints1 = [inv(tr(:,:,linkIndex+1)) * [meshPoints, ones(size(meshPoints, 1), 1)]']';
    transformedPoints = transformedPoints1(:,1:3);
    algebraicDist = GetAlgebraicDist(transformedPoints, [0,0,0], radii);
    pointsInside = find(algebraicDist < 1);
    pointsInsideCount = numel(pointsInside);
    %disp(['From ', objectName, ' there are ', num2str(numel(pointsInside)), ' points inside the ', num2str(linkIndex), 'th ellipsoid']);
end

function checkForEStop(serialObj)
    if serialObj.NumBytesAvailable > 0  % Check if data is available
        data = readline(serialObj);
        if strcmp(data, 'Stop')
            serialObj.UserData.hardwareEstopPress = true;
            disp("Button Pressed: Emergency Stop Activated");
        end
    end
end

% function waitForButtonPress(src)
%     % Read the ASCII data from the serialport object.
%     data = readline(src);
% 
%     % Check if the data corresponds to a button press (in this case, "Stop").
%     if strcmp(data, 'Stop')
%         hardwareEstopPress = true;
%         % Display that a button has been pressed
%         %disp("Button Pressed");
%     end
% end
%{
function tr = updateEllipsoids(tr, qMat, stepIndex, ellipsoidHandles, radiiList, L)
    % Loop through each link and update the transformation and ellipsoid
    numLinks = numel(L); % Number of links in the robot model
    
    for i = 1:numLinks
        % Recalculate transformation for the current link
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(stepIndex, i)) * transl(0, 0, L(i).d) * ...
                      transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        
        % Delete the previous ellipsoid handle if it exists
        try delete(ellipsoidHandles(i)); end;

        % Get radii for the current link's ellipsoid
        radii = radiiList(i, :);

        % Compute the center point from the transformation matrix
        centerPoint = tr(1:3, 4, i+1)';

        % Create ellipsoid data
        [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3));

        % Apply transformation to the ellipsoid points
        transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
        X_transformed = reshape(transformedPoints(1, :), size(X));
        Y_transformed = reshape(transformedPoints(2, :), size(Y));
        Z_transformed = reshape(transformedPoints(3, :), size(Z));

        % Update the plot (refresh each ellipsoid to its new position)
        ellipsoidHandles(i) = surf(X_transformed, Y_transformed, Z_transformed, ...
                                   'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
end
%}

%{
function updateEllipsoids(tr, qMat, stepIndex, i, ellipsoidHandles, radiiList, L) 
        % Recalculate transformations for each link in UR3 for each step
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(stepIndex, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
        % Delete the previous ellipsoids if they exist
        try delete(ellipsoidHandles(i)); end;
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
end
%}