%% DIPC Simulation Setup

close all; % Close any existing figures

% --- 1. System Definition ---
% Define physical parameters of the system
M = 1.5; m1 = 0.5; m2 = 0.75;
l1 = 0.5; l2 = 0.75; g = 9.81;

%Friction
b_friction = 0.9;

%chart Boundarys
x_min = -2.5;
x_max = 2.5; 

% Load results from previous skript
load("Funwork1_Gery_Matrix.mat")
disp('Jacobian Matrix A:');
disp(A);
disp('Jacobian Matrix B:');
disp(B);



A(4,4) = -b_friction; % Damping on cart velocity (x_dot)
A(5,5) = -b_friction; % Damping on pendulum 1 velocity (theta1_dot)
A(6,6) = -b_friction; % Damping on pendulum 2 velocity (theta2_dot)

%Simtime and steps
tfinal = 4;       % seconds
dt = 0.01;         % Time steps

% Initial Conditions
x_pos_init      = 0;
theta1_init     = 0.01;
theta2_init     = 0.02;
x_dot_init      = 0;
theta1_dot_init = 0;
theta2_dot_init = 0;

% Initialize the state vector for the loop
x_vec = [x_pos_init; theta1_init; theta2_init; x_dot_init; theta1_dot_init; theta2_dot_init];

% --- 3. Graphics Initialization ---
% Define cart dimensions for plotting
cart_width = 0.5;
cart_height = 0.25;

% Set up the figure and axis
figure;
ax = gca;
axis([-3 3 -1.5 1.5]);
grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('Double Inverted Pendulum Simulation');

% Calculate initial positions
x_cart = x_vec(1);
y_cart = 0;
x_m1 = x_cart + l1 * sin(x_vec(2));
y_m1 = l1 * cos(x_vec(2));
x_m2 = x_m1 + l2 * sin(x_vec(3));
y_m2 = y_m1 + l2 * cos(x_vec(3));

% Create graphics objects
cart = rectangle(ax, 'Position', [x_cart - cart_width/2, y_cart - cart_height/2, cart_width, cart_height], 'FaceColor', [0.5 0.5 0.5]);
xline(ax, x_min, 'k--', 'LineWidth', 2); % Left boundary line
xline(ax, x_max, 'k--', 'LineWidth', 2); % Right boundary line

bar1 = line(ax, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1], 'LineWidth', 3, 'Color', 'b');
mass1 = line(ax, 'XData', x_m1, 'YData', y_m1, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
bar2 = line(ax, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2], 'LineWidth', 3, 'Color', 'b');
mass2 = line(ax, 'XData', x_m2, 'YData', y_m2, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
time_text = text(ax, -2.8, 2.2, 'Time: 0.00s', 'FontSize', 14);

% --- 4. Video Setup ---
vidObj = VideoWriter('DIPC_SIm_Gery.mp4', 'MPEG-4');
vidObj.FrameRate = 1/dt;
open(vidObj);

% --- 5. Simulation Loop ---
for t = 0:dt:tfinal
    
    % Calculate the state derivative using the linearized model (u=0)
    x_dot = A * x_vec;
    
    % Update the state using Forward Euler integration
    x_vec = x_vec + x_dot * dt;

    if x_vec(1) >= x_max % Check right boundary
        x_vec(1) = x_max; % Set position to the boundary
        x_vec(4) = 0;     % Set horizontal velocity to zero
    elseif x_vec(1) <= x_min % Check left boundary
        x_vec(1) = x_min; % Set position to the boundary
        x_vec(4) = 0;     % Set horizontal velocity to zero
    end
    
    % --- Update Graphics ---
    % Extract current positions
    x_cart = x_vec(1);
    theta1 = x_vec(2);
    theta2 = x_vec(3);
    
    % Calculate new coordinates for plotting
    x_m1 = x_cart + l1 * sin(theta1);
    y_m1 = l1 * cos(theta1);
    x_m2 = x_m1 + l2 * sin(theta2);
    y_m2 = y_m1 + l2 * cos(theta2);
    
    % Update the properties of the graphics objects
    set(cart, 'Position', [x_cart - cart_width/2, y_cart - cart_height/2, cart_width, cart_height]);
    set(bar1, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1]);
    set(mass1, 'XData', x_m1, 'YData', y_m1);
    set(bar2, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2]);
    set(mass2, 'XData', x_m2, 'YData', y_m2);
    set(time_text, 'String', sprintf('Time: %.2fs', t));
    
    drawnow; % Force MATLAB to draw the updated figure
    
    % Write the current frame to the video file
    frame = getframe(gcf);
    writeVideo(vidObj, frame);
end

close(vidObj);
