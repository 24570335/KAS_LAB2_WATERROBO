hold on
kitchenEnvironment;

r = UR3e_adjusted;
waypoints = [71*pi/180,0,0,0,0,pi/4;
             pi,-1/12,1/12,-pi/2,3*pi/2,0;
             2*pi*7/18,-1/12,1/12,-pi/2,2*pi,0;
             pi/4,0,0,pi/-2,2*pi,0;];

br = BrushBot;
waypoints2 = [0,-pi/2,0,0,0;
             0,-109*pi/90,pi*19/90,0,0;
             0,-pi/2,0,0,0;
             0,-109*pi/90,pi*19/90,0,0;];

grip = Gripper;
% Testing gripper ply files:
PlaceObject('GripperLink5.ply',[0,-1.5,0.15]);
PlaceObject('GripperLink3.ply',[0,-0.5,0.15]);
PlaceObject('GripperLink1.ply',[0,-1,0.15]);

q0 = [0,0,0,0,0,0];
q0_brush = [0,0,0,0,0];
steps = 25;

% Moving UR3e to bottle
q1 = waypoints(1,:);
qMat = jtraj(q0, q1, steps);
for j=1:steps
    r.model.animate(qMat(j, :)) % Animating the movement to bottle
    drawnow;
end

% Moving UR3e to sink
q2 = waypoints(2,:);
qMat = jtraj(q1,q2,steps);
for k=1:steps
    r.model.animate(qMat(k,:))% Animating the movement to sink
    drawnow;
end

% Moving UR3e to tipping position
q3 = waypoints(3,:);
qMat = jtraj(q2,q3,steps);
for l=1:steps
    r.model.animate(qMat(l,:)) % Animating the movement of a tipping position
    drawnow;
end

% Moving UR3e to drying station
q4 = waypoints(4,:);
qMat = jtraj(q3,q4,steps);
for m=1:steps
    r.model.animate(qMat(m,:)) % Animating the movement to dry bottle
    drawnow;
end

