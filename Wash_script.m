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
waypoints = [71*pi/180,0,0,0,0,0;
             pi,-1/12,1/12,-pi/2,3*pi/2,-pi/2;
             2*pi*7/18,-1/12,1/12,-pi/2,2*pi,0;
             pi/4,0,0,pi/-2,2*pi,0;];

br = BrushBot;
waypoints2 = [0,-pi/2,0,0,0;
             0,-109*pi/90,pi*19/90,0,0;
             0,-pi/2,0,0,0;
             0,-pi/2,0,0,0];


grip = Gripper;
% grip.model.base = br.model.fkine(br.model.getpos());
qopen = [0,0.2,0.4];
qclose = [0,0.1,0.2];

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

