%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Autor: Anhelina Kyrylova                            %%%%%%%%%%%%%%%%
%%%%%% MATLAB script to implement a simple PID control     %%%%%%%%%%%%%%%%
%%%%%% algorithm for controlling the position of a robot.  %%%%%%%%%%%%%%%%
%%%%%% Simulate the robot's motion based on a desired      %%%%%%%%%%%%%%%%
%%%%%% position and generate control signals to adjust     %%%%%%%%%%%%%%%%
%%%%%% the robot's position towards the desired value.     %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% System parameters
Kp = 1.0;   % Proportional gain
Ki = 0.1;   % Integral gain
Kd = 0.05;  % Derivative gain

% Desired position
desired_position = 90;  % Desired robot position

% Initial values
current_position = 0;   % Current robot position
prev_error = 0;         % Previous error
integral_error = 0;     % Integral error

% Function to simulate robot motion
simulate_robot_motion = @(t) desired_position + 10 * sin(2*pi*t/20);

% Control loop
for t = 1:100
    % Measurement of current position (simulation)
    measurement = simulate_robot_motion(t);
    
    % Calculation of error
    error = desired_position - measurement;
    
    % Calculation of integral error (simple sum)
    integral_error = integral_error + error;
    
    % Calculation of derivative error (difference between current and previous error)
    derivative_error = error - prev_error;
    
    % Control signal (sum of proportional, integral, and derivative errors)
    control_signal = Kp*error + Ki*integral_error + Kd*derivative_error;
    
    % Limiting the control signal (in this example, it is limited to the range from -10 to 10)
    control_signal = max(-10, min(control_signal, 10));
    
    % Applying the control signal to the robot
    % For example, just updating the current position by adding the control signal
    current_position = current_position + control_signal;
    
    % Displaying the current position and control signal
    disp(['Current robot position: ', num2str(current_position)]);
    disp(['Control signal: ', num2str(control_signal)]);
    
    % Updating the previous error
    prev_error = error;
    
    % Pause to simulate real-time
    pause(0.1);
end
