% %% DIPC Simulation Setup
% clc;
% clear;
close all; % Close any existing figures


%  physical parameters
l1 = 0.5; 
l2 = 0.75; 
g = 9.81;

%Friction addtional playaround
b_friction = 0;

%chart Boundarys to keep the chart in the Figure
x_min = -2.5;
x_max = 2.5; 

%External Force 
u=0;

%Simtime and steps
tfinal = 5;       % seconds
dt = 0.05;         % Time steps


% EOM --> x 
eq1 = (M + m1 + m2) * x_ddot ...
      + (m1 + m2) * l1 * theta1_ddot * cos(theta1) ...
      + m2 * l2 * theta2_ddot * cos(theta2) ...
      - (m1 + m2) * l1 * theta1_dot^2 * sin(theta1) ...
      - m2 * l2 * theta2_dot^2 * sin(theta2) ...
      - u == 0;

% EOM --> theta1 
eq2 = (m1 + m2) * l1^2 * theta1_ddot...
      + (m1 + m2) * l1 * x_ddot * cos(theta1) ...
      + m2 * l1 * l2 * theta2_ddot * cos(theta1 - theta2) ...
      + m2 * l1 * l2 * theta2_dot^2 * sin(theta1 - theta2) ... 
      - (m1 + m2) * g * l1 * sin(theta1) == 0;

% EOM --> theta2 
eq3 = m2 * l2^2 * theta2_ddot ...
      + m2 * l2 * x_ddot * cos(theta2) ...
      + m2 * l1 * l2 * theta1_ddot * cos(theta1 - theta2) ...
      - m2 * l1 * l2 * theta1_dot^2 * sin(theta1 - theta2) ... 
      - m2 * g * l2 * sin(theta2) == 0;

%Solve equations for higthes derviative
eqddot = solve([eq1, eq2, eq3], [x_ddot, theta1_ddot, theta2_ddot]);


% state-derivative vector f(x,u)
f = [x_dot;
     theta1_dot;
     theta2_dot;
     eqddot.x_ddot
     eqddot.theta1_ddot;
     eqddot.theta2_ddot;];


% Initial Conditions
x_pos_init      = 0;
theta1_init     = 0.01;
theta2_init     = 0.02;
x_dot_init      = 0;
theta1_dot_init = 0;
theta2_dot_init = 0;

% Init  state vector for the loop
x_vec = [x_pos_init; theta1_init; theta2_init; x_dot_init; theta1_dot_init; theta2_dot_init];

% Cart dimensions 
cart_width = 0.5;
cart_height = 0.25;

% Set up the figure and axis
figure;
ax = gca;
axis([-3 3 -1.5 1.5]);
grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('Double Inverted Pendulum on Cart Simulation');

% Calculate initial positions
x_cart = x_vec(1);
y_cart = 0;
x_m1 = x_cart + l1 * sin(x_vec(2));
y_m1 = l1 * cos(x_vec(2));
x_m2 = x_m1 + l2 * sin(x_vec(3));
y_m2 = y_m1 + l2 * cos(x_vec(3));

% Create graphics objects
cart = rectangle(ax, 'Position', [x_cart - cart_width/2, y_cart - cart_height, cart_width, cart_height], 'FaceColor', [0.5 0.5 0.5]);
xline(ax, x_min, 'k--', 'LineWidth', 2); % Left boundary line
xline(ax, x_max, 'k--', 'LineWidth', 2); % Right boundary line

bar1 = line(ax, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1], 'LineWidth', 3, 'Color', 'b');
mass1 = line(ax, 'XData', x_m1, 'YData', y_m1, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
bar2 = line(ax, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2], 'LineWidth', 3, 'Color', 'b');
mass2 = line(ax, 'XData', x_m2, 'YData', y_m2, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
time_text = text(ax, -2.8, 2.2, 'Time: 0.00s', 'FontSize', 14);

% start Video Setup
vidObj = VideoWriter('DIPC_Sim_Gery.mp4', 'MPEG-4');
vidObj.FrameRate = 1/dt;
open(vidObj);

% Simulation using ode45 
fprintf('Running simulation...\n');

% Define the time span for the simulation
tspan = [0 tfinal];

x_vec_init = [x_pos_init; theta1_init; theta2_init; x_dot_init; theta1_dot_init; theta2_dot_init];

system_dynamics = @(t, x) A_clu2*x+B_u2;

[T, X] = ode45(system_dynamics, tspan, x_vec_init);

fprintf('Simulation complete. Starting animation...\n');

% time vector with fixed 
t_anim = (0:dt:tfinal)';

% Interpolate the results from ode45 
X_anim = interp1(T, X, t_anim);

% Loop through states t
for k = 1:length(t_anim)
    
    % Get the state vector 
    x_vec = X_anim(k, :)'; 

    % Boundary Collision (Optional)
    if x_vec(1) >= x_max 
        x_vec(1) = x_max; 
        x_vec(4) = 0;     
    elseif x_vec(1) <= x_min 
        x_vec(1) = x_min; 
        x_vec(4) = 0;    
    end
    
    % Update Graphics
    x_cart = x_vec(1);
    theta1 = x_vec(2);
    theta2 = x_vec(3);
    
    x_m1 = x_cart + l1 * sin(theta1);
    y_m1 = l1 * cos(theta1);
    x_m2 = x_m1 + l2 * sin(theta2);
    y_m2 = y_m1 + l2 * cos(theta2);
    
    set(cart, 'Position', [x_cart - cart_width/2, -cart_height, cart_width, cart_height]);
    set(bar1, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1]);
    set(mass1, 'XData', x_m1, 'YData', y_m1);
    set(bar2, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2]);
    set(mass2, 'XData', x_m2, 'YData', y_m2);
    set(time_text, 'String', sprintf('Time: %.2fs', t_anim(k)));
    
    drawnow; 
    
    % current frame to the video file
    frame = getframe(gcf);
    writeVideo(vidObj, frame);
end

% Close the video file object
close(vidObj);
disp('Video file saved.');