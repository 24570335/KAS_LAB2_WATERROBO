function BrushBot_GUI
    % Initialize the BrushBot robot object
    bot = BrushBot();  % Assuming no input is needed for basic initialization

    % Create the GUI figure
    fig = figure('Name', 'BrushBot Controller', 'NumberTitle', 'off', ...
        'Position', [100, 100, 800, 600]);

    % Axes for robot visualization
    ax = axes('Parent', fig, 'Position', [0.3, 0.2, 0.65, 0.75]);
    axes(ax);  % Set these axes as active
    bot.model.plot(zeros(1, bot.model.n));  % Initial plot on the active axes

    % Create buttons for joint control
    buttonPanel = uipanel('Parent', fig, 'Title', 'Joint Controls', ...
        'Position', [0.05, 0.2, 0.2, 0.75]);

    numJoints = bot.model.n;
    stepSize = pi / 18;  % Increment size (10 degrees in radians)
    jointAngles = zeros(1, numJoints);  % Store joint angles

    for i = 1:numJoints
        % Increment button
        uicontrol('Parent', buttonPanel, 'Style', 'pushbutton', ...
            'String', sprintf('Joint %d +', i), ...
            'Position', [10, 300 - 60 * i, 70, 40], ...
            'Callback', @(src, ~) adjustJoint(i, stepSize));

        % Decrement button
        uicontrol('Parent', buttonPanel, 'Style', 'pushbutton', ...
            'String', sprintf('Joint %d -', i), ...
            'Position', [90, 300 - 60 * i, 70, 40], ...
            'Callback', @(src, ~) adjustJoint(i, -stepSize));
    end

    % Reset button
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Reset', ...
        'Position', [50, 50, 100, 40], ...
        'Callback', @(~, ~) resetRobot());

    % Display for joint angles and end-effector position
    statusPanel = uipanel('Parent', fig, 'Title', 'Status', ...
        'Position', [0.05, 0.05, 0.9, 0.1]);

    jointAnglesText = uicontrol('Parent', statusPanel, 'Style', 'text', ...
        'Position', [10, 10, 400, 30], 'HorizontalAlignment', 'left');
    endEffectorText = uicontrol('Parent', statusPanel, 'Style', 'text', ...
        'Position', [420, 10, 400, 30], 'HorizontalAlignment', 'left');

    % Nested function to adjust the joint angle
    function adjustJoint(jointIndex, step)
        jointAngles(jointIndex) = jointAngles(jointIndex) + step;  % Update angle
        bot.model.animate(jointAngles);  % Update robot visualization
        
        % Update joint angles and end-effector position display
        set(jointAnglesText, 'String', sprintf('Joint Angles: %s', mat2str(jointAngles, 3)));
        T = bot.model.fkine(jointAngles);  % Forward kinematics for end-effector pose
        set(endEffectorText, 'String', sprintf('End-Effector Pose: %s', mat2str(T.t, 3)));
    end

    % Nested function to reset the robot to the home configuration
    function resetRobot()
        jointAngles = zeros(1, numJoints);  % Reset joint angles
        bot.model.animate(jointAngles);  % Reset robot visualization
        
        % Update status display
        set(jointAnglesText, 'String', 'Joint Angles: [0, 0, 0, 0, 0, 0]');
        T = bot.model.fkine(jointAngles);
        set(endEffectorText, 'String', sprintf('End-Effector Pose: %s', mat2str(T.t, 3)));
    end
end
