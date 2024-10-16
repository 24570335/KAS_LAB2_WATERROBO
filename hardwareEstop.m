% Close all existing serialport connections
delete(serialportfind);


% Manually specify the port you verified
port_name = '/dev/cu.usbserial-14440';  % Adjust this if the port name is different
baud_rate = 9600;  % Match the baud rate with the Arduino code

% Create the serial port object manually with the correct port and baud rate
serialObj = serialport(port_name, baud_rate)

%{
% Check if any port is available
if isempty(port_number)
    error('No serial port found. Please connect your Arduino.');
else
    % Connect to the Arduino Due by creating a serialport object.
    % Use the port and baud specified in the Arduino code.
    serialObj = serialport('/dev/cu.usbserial-14440',9600);
end
%}

%Set the Terminator property to match the terminator that you specified in the Arduino code.
configureTerminator(serialObj,"CR/LF");

%Flush the serialport object to remove any old data.
flush(serialObj);

%Prepare the UserData property to store the Arduino data. 
% In this case, define UserData to be a struct in which the 
% Data field contains sine wave values and Count records the number of data points collected.
serialObj.UserData = struct("Data",[],"Count",1);

% Set the callback to trigger on the terminator (CR/LF)
configureCallback(serialObj, "terminator", @(src, event) waitForButtonPress(src));


function waitForButtonPress(src)
    % Read the ASCII data from the serialport object.
    data = readline(src);

    % Check if the data corresponds to a button press (in this case, "Stop").
    if strcmp(data, 'Stop')
        % Display that a button has been pressed
        disp("Button Pressed");
    end
end