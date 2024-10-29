%To initialise a connection to the ROS computer from Matlab, call rosinit to the correct IP address, and then start 
%To subscribe to the joint states
rosinit('192.168.27.1'); % If unsure, please ask a tutor
%%
% load('ur3_q.mat');
r = UR3e_adjusted;


jointAngles = [71*pi/180,-pi/6,0,0,0,0;
             pi,-0.452,0,0,0,pi/2;
             2*pi*7/18,-pi/6,1/12,-pi/2,0,pi/2;
             pi/4,-pi/6,0,pi/-2,0,pi/2;];


%jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');

%{
the ROS computer is fully booted. If you are using the Pi, this may take up to 2 minutes from powering it on.
your computer is not on a separate network internet (for some people it will work even if they are on the internet via WIFI, but for many, it will not)
you don't have a firewall that is blocking all traffic on this new network
%}

%To get the current joint state from the real robot
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

%If this fails then try to see what is in the latest message. If it is empty then you are not connected properly.
jointStateSubscriber.LatestMessage

%Before sending commands, we create a variable with the joint names so that the joint commands are associated 
% with a particular joint.
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%{
To send a set of joint angles to the robot, you need to create a 'client' and define a 'goal'. The function rosactionclient 
Links to an external site. will create these variables, and allow you to use them to "connect to an action server using a 
SimpleActionClient object and request the execution of action goals."
%}

%In the following example, we are rotating the first (shoulder_pan) and last (wrist_3) joints by pi/8 in 5 seconds.
[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 10; % This is how many seconds the movement will take

%To use the RG2 Gripper, first create the ROS Service clients for the open and close services
openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

%Then you can call the service to open the gripper
openService.call();
pause(1);

for i = 1
    q = jointAngles(i, :);
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
      
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = q;
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);

    goal.Trajectory.Points = [startJointSend; endJointSend];

    %Then you can send this to the robot
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    sendGoal(client,goal);

    pause(durationSeconds + bufferSeconds);

    currentJointState_123456 = q;
end



%Similarly, you can call the service to close the gripper
 closeService.call();
 pause(1);

% continue with trajectory of moving bottle to sink, tipping it and then
% placing it back
for i = 2:4
    q = jointAngles(i, :);
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
      
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = q;
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(10);

    goal.Trajectory.Points = [startJointSend; endJointSend];

    %Then you can send this to the robot
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    sendGoal(client,goal);

    pause(durationSeconds + bufferSeconds);

    currentJointState_123456 = q;


end

%Then you can call the service to open the gripper
openService.call();
pause(1);
