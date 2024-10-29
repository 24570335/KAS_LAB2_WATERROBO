function BrushBot_GUI
    % Initialize the GUI and BrushBot model
    close all;

    % Create the GUI window
    fig = uifigure('Name', 'BrushBot Control', 'Position', [100 100 700 700]);

    % Initialize the BrushBot model and store it in the figure's UserData
    brushBot = BrushBot();  % Create an instance of BrushBot
    brushBot.CreateModel(); % Ensure the model is initialized properly

    % Set the base transformation matrix to place it at the origin (0, 0, 0)
    brushBot.model.base = transl(0, 0, 0);  % Set base at origin

    % Store the robot object in UserData for later access
    fig.UserData.robot = brushBot;

    % Set an initial joint configuration (natural position)
    initialQ = zeros(1, brushBot.model.n);  % Adjust for the number of joints

    % Clear the figure to ensure only one plot is active
    clf;  % Clear the current figure

    % Plot the robot in the initial configuration with a defined workspace
    brushBot.model.plot(initialQ, 'workspace', [-1 1 -1 1 -1 1]);
    
    % Ensure the view is properly aligned
    view(3);  % Set to 3D view
    axis equal;  % Equal scaling along axes

    
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

    % Cartesian Movement Controls
    uilabel(fig, 'Position', [550 500 200 20], 'Text', 'Cartesian Controls');

    % X-axis Control
    uibutton(fig, 'Position', [550 450 100 40], 'Text', '+X', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([0.05, 0, 0]));
    uibutton(fig, 'Position', [660 450 100 40], 'Text', '-X', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([-0.05, 0, 0]));

    % Y-axis Control
    uibutton(fig, 'Position', [550 400 100 40], 'Text', '+Y', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([0, 0.05, 0]));
    uibutton(fig, 'Position', [660 400 100 40], 'Text', '-Y', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([0, -0.05, 0]));

    % Z-axis Control
    uibutton(fig, 'Position', [550 350 100 40], 'Text', '+Z', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([0, 0, 0.05]));
    uibutton(fig, 'Position', [660 350 100 40], 'Text', '-Z', ...
        'ButtonPushedFcn', @(~,~)move_cartesian([0, 0, -0.05]));

    % Helper Functions

    function activate_teach_mode()
        % Activate the teach mode using Peter Corke's Robotics Toolbox
        robot = fig.UserData.robot;
        robot.model.teach();  % Launch the teach GUI
    end

    function update_joint(jointIndex, jointValue)
        % Callback for updating joint values using sliders
        robot = fig.UserData.robot;

        % Get the current joint positions
        q = robot.model.getpos();  

        % Update the specified joint value
        q(jointIndex) = jointValue;

        % Animate the robot to the new position
        robot.model.animate(q);
        drawnow;  % Refresh the plot
    end

    function move_robot(direction)
        % Callback for D-pad buttons to move the robot
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
                q(1) = q(1) + stepSize;  % Rotate base joint left
            case 'right'
                q(1) = q(1) - stepSize;  % Rotate base joint right
        end

        % Animate the robot to the new position
        robot.model.animate(q);
        drawnow;  % Refresh the plot
    end

function move_cartesian(delta)
    % Callback for Cartesian movement buttons
    robot = fig.UserData.robot;

    % Get the current joint configuration
    currentQ = robot.model.getpos();

    % Debugging: Print the current joint configuration
    disp('Current joint configuration:');
    disp(currentQ);

    % Calculate the end-effector pose using fkine
    try
        currentPose = robot.model.fkine(currentQ);
    catch ME
        disp('Error calculating fkine:');
        disp(ME.message);
        return;  % Exit if fkine fails
    end

    % Verify that the returned pose is a 4x4 matrix
    if ~isequal(size(currentPose), [4, 4])
        error('fkine did not return a valid 4x4 transformation matrix.');
    end

    % Extract the translation component
    T = currentPose(1:3, 4);

    % Apply the delta translation
    newT = T + delta(:);

    % Update the transformation matrix with the new translation
    newPose = currentPose;
    newPose(1:3, 4) = newT;

    % Calculate the joint angles for the new pose using inverse kinematics
    newQ = robot.model.ikcon(newPose, currentQ);

    % Animate the robot to the new configuration
    robot.model.animate(newQ);
    drawnow;  % Refresh the plot
end






end
