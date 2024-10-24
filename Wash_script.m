hold on
kitchenEnvironment;
r = UR3e_adjusted;
    waypoints = [71*pi/180,0,0,0,0,pi/4;
                 pi,-1/12,1/12,-pi/2,3*pi/2,0;
                 2*pi*7/18,-1/12,1/12,-pi/2,2*pi,0;
                 pi/4,0,0,pi/-2,2*pi,0;];

    waypoints2 = [0,-pi/2,0,0,0;
                 0,-109*pi/90,pi*19/90,0,0;
                 0,-pi/2,0,0,0;
                 0,-109*pi/90,pi*19/90,0,0;];
br = BrushBot;
q0 = [0,0,0,0,0,0];
q0_brush = [0,0,0,0,0]
steps = 25;

% q1 = waypoints(1,:);
% qMat = jtraj(q0, q1, steps);
% 
% for j=1:steps
%     r.model.animate(qMat(j, :)) % move to bottle
%     drawnow;
% end
% q2 = waypoints(2,:);
% qMat = jtraj(q1,q2,steps);
% 
% for k=1:steps
%     r.model.animate(qMat(k,:))% move to sink
%     drawnow;
% end
% 
% q3 = waypoints(3,:);
% qMat = jtraj(q2,q3,steps);
% 
% for l=1:steps
%     r.model.animate(qMat(l,:)) % tip
%     drawnow;
% end
% q4 = waypoints(4,:);
% qMat = jtraj(q3,q4,steps);
% 
% for m=1:steps
%     r.model.animate(qMat(l,:))%to dry
%     drawnow;
% end

