function ur3e_gui()
    % Create a figure for the GUI
    f = figure('Position', [100, 100, 300, 300], 'Name', 'UR3e Control', 'NumberTitle', 'off');

    % Create buttons for movement
    uicontrol('Style', 'pushbutton', 'String', 'Move Up', ...
              'Position', [50, 220, 200, 40], ...
              'Callback', @moveUp);
    
    uicontrol('Style', 'pushbutton', 'String', 'Move Down', ...
              'Position', [50, 170, 200, 40], ...
              'Callback', @moveDown);
          
    uicontrol('Style', 'pushbutton', 'String', 'Move Left', ...
              'Position', [50, 120, 200, 40], ...
              'Callback', @moveLeft);
          
    uicontrol('Style', 'pushbutton', 'String', 'Move Right', ...
              'Position', [50, 70, 200, 40], ...
              'Callback', @moveRight);
          
    uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
              'Position', [50, 20, 200, 40], ...
              'Callback', @stopMovement);
    
    % Callback functions for button presses
    function moveUp(~, ~)
        % Send command to move the arm up
        disp('Moving Up');
        % add command to control 1
    end

    function moveDown(~, ~)
        % Send command to move the arm down
        disp('Moving Down');
        % add command to control 2
    end

    function moveLeft(~, ~)
        % Send command to move the arm left
        disp('Moving Left');
        % add command to control3
    end

    function moveRight(~, ~)
        % Send command to move the arm right
        disp('Moving Right');
        % add command to control 4
    end

    function stopMovement(~, ~)
        % Send command to stop the arm
        disp('Stopping Movement');
        % add command to control 5
    end
end
