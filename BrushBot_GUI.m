function BrushBot_GUI
    % Initialize BrushBot robot and create GUI
    close all;

    % Create the GUI window
    fig = uifigure('Name', 'BrushBot Control', 'Position', [100 100 700 700]);

    % Initialize BrushBot
    brushBot = BrushBot();
    brushBot.model.plot(zeros(1, brushBot.model.n));  % Initial pose

    % Store robot in UserData for access in callbacks
    fig.UserData.robot = brushBot;

    % Create Sliders for Joint Control
    numJoints = brushBot.model.n;  % Number of joints in the robot
    for i = 1:numJoints
        uilabel(fig, 'Position', [50, 630 - (i * 50), 100, 20], ...
            'Text', sprintf('Joint %d', i));

        uislider(fig, 'Position', [150, 630 - (i * 50), 300, 3], ...
            'Limits', [-pi, pi], ...
            'Value', 0, ...
            'ValueChangedFcn', @(sld, ~)update_joint(i, sld.Value));
    end

    % Place D-Pad Style Controls below the sliders
    uibutton(fig, 'Position', [240 150 60 40], 'Text', 'Up', ...
        'ButtonPushedFcn', @(~,~)move_robot('up'));

    uibutton(fig, 'Position', [180 100 60 40], 'Text', 'Left', ...
        'ButtonPushedFcn', @(~,~)move_robot('left'));

    uibutton(fig, 'Position', [240 100 60 40], 'Text', 'Down', ...
        'ButtonPushedFcn', @(~,~)move_robot('down'));

    uibutton(fig, 'Position', [300 100 60 40], 'Text', 'Right', ...
        'ButtonPushedFcn', @(~,~)move_robot('right'));

    % Place Gripper Control Buttons next to the D-Pad
    uibutton(fig, 'Position', [380 150 100 40], 'Text', 'Open Gripper', ...
        'ButtonPushedFcn', @(~,~)control_gripper('open'));

    uibutton(fig, 'Position', [380 100 100 40], 'Text', 'Close Gripper', ...
        'ButtonPushedFcn', @(~,~)control_gripper('close'));

    function update_joint(jointIndex, jointValue)
        % Callback for updating joint values using sliders
        robot = fig.UserData.robot;

        % Get the current joint positions
        q = robot.model.getpos();  

        % Update the specific joint value
        q(jointIndex) = jointValue;

        % Animate the robot to the new position
        robot.model.animate(q);
        drawnow;  % Refresh the plot
    end

    function move_robot(direction)
        % Callback for directional movement buttons
        robot = fig.UserData.robot;
        q = robot.model.getpos();  % Get current joint configuration

        % Define the step size for movement
        stepSize = 0.1;

        % Adjust the joint angles based on the direction
        switch direction
            case 'up'
                q(2) = q(2) + stepSize;  % Move joint 2 upwards
            case 'down'
                q(2) = q(2) - stepSize;  % Move joint 2 downwards
            case 'left'
                q(1) = q(1) + stepSize;  % Rotate base left
            case 'right'
                q(1) = q(1) - stepSize;  % Rotate base right
        end

        % Animate the robot to the new position
        robot.model.animate(q);
        drawnow;
    end

    function control_gripper(action)
        % Callback for gripper control buttons
        % Define joint configurations for open and closed states
        qOpen = [0.5];  % Example open state value (adjust as needed)
        qClose = [0];   % Example closed state value (adjust as needed)

        % Execute the appropriate action
        switch action
            case 'open'
                brushBot.model.animate(qOpen);  % Animate to open state
            case 'close'
                brushBot.model.animate(qClose);  % Animate to closed state
        end

        drawnow;  % Refresh the GUI
    end



    % function GripperControl
    %     % Initialize Gripper object
    %     gripper = Gripper(transl(0, 0, 0));  % Initialize at origin
    % 
    %     % Plot the gripper in its initial position
    %     gripper.model.plot([0]);  % Assuming it has one joint for control
    % 
    %     % Create a simple GUI for controlling the gripper
    %     fig = uifigure('Name', 'Gripper Control', 'Position', [100 100 300 200]);
    % 
    %     % Create Open Gripper button
    %     uibutton(fig, 'Position', [50 120 100 40], 'Text', 'Open Gripper', ...
    %         'ButtonPushedFcn', @(~,~)control_gripper(gripper, 'open'));
    % 
    %     % Create Close Gripper button
    %     uibutton(fig, 'Position', [150 120 100 40], 'Text', 'Close Gripper', ...
    %         'ButtonPushedFcn', @(~,~)control_gripper(gripper, 'close'));
    % 
    %     % Display the gripper's joint state label
    %     jointStateLabel = uilabel(fig, 'Position', [50 70 200 20], ...
    %         'Text', 'Gripper State: Closed');
    % 
    %     % Store the label's handle to update it dynamically
    %     fig.UserData.jointStateLabel = jointStateLabel;
    % end
    % 
    % function control_gripper(gripper, action)
    %     % Control function to open/close the gripper
    %     % Define joint configurations for open and closed states
    %     qOpen = [0.5];  % Example open state value (adjust as needed)
    %     qClose = [0];   % Example closed state value (adjust as needed)
    % 
    %     % Execute the appropriate action
    %     switch action
    %         case 'open'
    %             gripper.model.animate(qOpen);  % Animate to open state
    %             update_label('Open');  % Update GUI label
    % 
    %         case 'close'
    %             gripper.model.animate(qClose);  % Animate to closed state
    %             update_label('Closed');  % Update GUI label
    %     end
    % 
    %     drawnow;  % Refresh the GUI
    % end
    % 
    % function update_label(state)
    %     % Update the joint state label in the GUI
    %     fig = gcf;  % Get current figure handle
    %     label = fig.UserData.jointStateLabel;  % Access the label handle
    %     label.Text = ['Gripper State: ', state];  % Update label text
    % end

end
