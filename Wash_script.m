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

%% Creating ellipsoids on robot
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
    [X, Y, Z] = ellipsoid(-0.01, 0, 0, radii(1), radii(2), radii(3), 20);
    transformedPoints = tr(:,:,i+1) * [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))]; %used to make homogenous matrix, be able to transform all points in one go (list)
    X_transformed = reshape(transformedPoints(1, :), size(X));
    Y_transformed = reshape(transformedPoints(2, :), size(Y));
    Z_transformed = reshape(transformedPoints(3, :), size(Z));
    surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
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
    br.model.animate(qMatb(j,:))
    grip.model.base = br.model.fkine(qMatb(j,:));
    grip.model.animate([0,0,0]);
    % grip.model.animate(qmatopen(j,:))


    % Recalculate transformations for each link in UR3 for each step
    tr(:,:,1) = r.model.base;
    for i = 1:r.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qMat(j, i)) * transl(0, 0, L(i).d) * transl(L(i).a/1.3, 0, 0) * trotx(L(i).alpha);
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
        surf(X_transformed, Y_transformed, Z_transformed, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
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

    drawnow()
end


% Moving UR3e to tipping position
q3 = waypoints(3,:);
qMat = jtraj(q2,q3,steps);
q3b = waypoints2(3,:);
qMatb= jtraj(q2b,q3b,steps);
for l=1:steps
    r.model.animate(qMat(l,:)) % Animating the movement of a tipping position

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

    
    drawnow()
end

% Moving UR3e to drying station
q4 = waypoints(4,:);
qMat = jtraj(q3,q4,steps);
q4b = waypoints2(4,:);
qMatb= jtraj(q3b,q4b,steps);
for m=1:steps
    r.model.animate(qMat(m,:)) % Animating the movement to dry bottle

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
    
    drawnow()
end

