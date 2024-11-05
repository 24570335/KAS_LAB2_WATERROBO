            % link(1) = Link('d', 0,    'a',0,      'alpha', pi/2 ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            % 
            % link(2) = Link([0     0       0       pi    1]);
            % link(2).qlim = [0 0.2];
            % 
            % link(3) = Link([0     0       0       0    1]);
            % link(3).qlim = [0.2 0.4];
            % gript = SerialLink(link,'name','grip'); 
            cowHerd = RobotCows(1);
ur5Robot = UR5;

cowHerd.cowModel{1}.base = ur5Robot.model.fkine([0,0,0,0,0,0])
cowHerd.cowModel{1}.animate(0)

qMatrix = jtraj([0,0,0,0,0,0],[pi/4,pi/4,pi/4,pi/4,pi/4,pi/4],50);


for i = 1:50
  ur5Robot.model.animate(qMatrix(i,:));
  cowHerd.cowModel{1}.base = ur5Robot.model.fkine(qMatrix(i,:));
  cowHerd.cowModel{1}.animate(0);
  drawnow()
end