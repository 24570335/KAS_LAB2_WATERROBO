function BrushBot_GUI
    % Initialize the GUI and BrushBot model
    close all;

    % Create the GUI window
    fig = uifigure('Name', 'BrushBot Control', 'Position', [100 100 700 700]);

    % Initialize the BrushBot model and store it in UserData
    brushBot = BrushBot();  
    brushBot.CreateModel();  

    % Store the BrushBot object and E-Stop state in UserData
    fig.UserData = struct('robot', brushBot, 'eStop', false);  

    % Set the base transformation at origin
    brushBot.model.base = transl(0, 0, 0);  

    % Create robot visualization figure
    robotFig = figure('Name', 'Robot View', 'NumberTitle', 'off');
    ax = axes('Parent', robotFig);  
    axes(ax);  
    brushBot.model.plot(zeros(1, brushBot.model.n), 'workspace', [-1 1 -1 1 -1 1]);

    disp('GUI and BrushBot initialized successfully.');

    % Add joint sliders
    numJoints = brushBot.model.n;  
    for i = 1:numJoints
        uilabel(fig, 'Position', [50, 630 - (i * 50), 100, 20], ...
            'Text', sprintf('Joint %d', i));
        uislider(fig, 'Position', [150, 630 - (i * 50), 300, 3], ...
            'Limits', [-pi, pi], 'Value', 0, ...
            'ValueChangedFcn', @(sld, ~)update_joint(i, sld.Value));
    end

    % Add E-Stop button
    uibutton(fig, 'Position', [50, 50, 100, 40], 'Text', 'E-Stop', ...
        'BackgroundColor', 'red', 'FontSize', 14, ...
        'ButtonPushedFcn', @(btn, ~)toggle_e_stop());

    % Add Teach Mode button
    uibutton(fig, 'Position', [50, 100, 100, 40], 'Text', 'Teach Mode', ...
        'ButtonPushedFcn', @(~,~)activate_teach_mode());

    % Add D-pad movement buttons
    uibutton(fig, 'Position', [240 150 60 40], 'Text', 'Up', ...
        'ButtonPushedFcn', @(~,~)move_robot('up'));
    uibutton(fig, 'Position', [180 100 60 40], 'Text', 'Left', ...
        'ButtonPushedFcn', @(~,~)move_robot('left'));
    uibutton(fig, 'Position', [240 100 60 40], 'Text', 'Down', ...
        'ButtonPushedFcn', @(~,~)move_robot('down'));
    uibutton(fig, 'Position', [300 100 60 40], 'Text', 'Right', ...
        'ButtonPushedFcn', @(~,~)move_robot('right'));

    % Cartesian movement controls
    add_cartesian_controls(fig);

    %% Helper Functions

    function toggle_e_stop()
        % Toggle E-Stop state in UserData
        fig.UserData.eStop = ~fig.UserData.eStop;
        if fig.UserData.eStop
            disp('E-Stop activated. Robot movements halted.');
        else
            disp('E-Stop released. Robot movements allowed.');
        end
    end

    function activate_teach_mode()
        % Activate teach mode
        robot = fig.UserData.robot;
        robot.model.teach();
    end

    function update_joint(jointIndex, jointValue)
        % Update joint value if E-Stop is not active
        if fig.UserData.eStop
            disp('E-Stop is active. Joint movement blocked.');
            return;
        end

        robot = fig.UserData.robot;
        q = robot.model.getpos();
        q(jointIndex) = jointValue;
        robot.model.animate(q);
    end

    function move_robot(direction)
        % Move robot based on D-pad input if E-Stop is not active
        if fig.UserData.eStop
            disp('E-Stop is active. Robot movement blocked.');
            return;
        end

        robot = fig.UserData.robot;
        q = robot.model.getpos();
        stepSize = 0.1;

        switch direction
            case 'up'
                q(2) = q(2) + stepSize;
            case 'down'
                q(2) = q(2) - stepSize;
            case 'left'
                q(1) = q(1) + stepSize;
            case 'right'
                q(1) = q(1) - stepSize;
        end

        robot.model.animate(q);
    end

    function move_cartesian(delta)
        % Move in Cartesian space if E-Stop is not active
        if fig.UserData.eStop
            disp('E-Stop is active. Cartesian movement blocked.');
            return;
        end

        robot = fig.UserData.robot;
        currentQ = robot.model.getpos();
        currentPose = robot.model.fkine(currentQ);

        newPose = currentPose * transl(delta);
        newQ = robot.model.ikcon(newPose, currentQ);
        robot.model.animate(newQ);
    end

    function add_cartesian_controls(fig)
        % Add Cartesian movement controls
        uibutton(fig, 'Position', [410 200 100 40], 'Text', '+X', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([0.05, 0, 0]));
        uibutton(fig, 'Position', [520 200 100 40], 'Text', '-X', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([-0.05, 0, 0]));
        uibutton(fig, 'Position', [410 150 100 40], 'Text', '+Y', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([0, 0.05, 0]));
        uibutton(fig, 'Position', [520 150 100 40], 'Text', '-Y', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([0, -0.05, 0]));
        uibutton(fig, 'Position', [410 100 100 40], 'Text', '+Z', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([0, 0, 0.05]));
        uibutton(fig, 'Position', [520 100 100 40], 'Text', '-Z', ...
            'ButtonPushedFcn', @(~,~)move_cartesian([0, 0, -0.05]));
    end
end
