function [T_log, X_log, U_log] = run_mpc_simulation(mpc_obj, f_handle_3u, C_u3, x_init, Ts, tfinal,r_target, xe, ue, video_filename)
% This function runs a closed-loop simulation of the non-linear DIPC
% model controlled by a given MPC controller.
%
% Inputs:
%   mpc_obj      - The MPC controller object
%   f_handle_3u  - Function handle for the non-linear dynamics
%   C_u3         - The output matrix C
%   x_init       - 6x1 initial state vector
%   Ts           - Sampling time
%   tfinal       - Total simulation time
%   video_filename - Name for the output .mp4 file
%
% Outputs:
%   T_log        - Time vector (Nx1)
%   X_log        - State history (Nx6)
%   U_log        - Input history (Nx3)

disp('Starting non-linear simulation...');

% --- Simulation Setup ---
t_steps = floor(tfinal / Ts); % Number of simulation steps
%r = [0; 0.2; 0]; % Reference for the outputs [x, theta1, theta2]

% Initialize the MPC state object
mpc_state = mpcstate(mpc_obj);
mpc_state.Plant = x_init - xe;

% Initialize logs for plotting and animation
x_true = x_init;
t = 0;
T_log = t;
X_log = x_true';
U_log = [0, 0, 0];


M= 1.5; m1= 0.5; m2 = 0.75;
l1 = 0.5; l2= 0.75; g = 9.81;

% --- Simulation Loop ---
for k = 1:t_steps
    % 1. Measure the "true" plant output from the current state
    y_true = C_u3 * x_true;
    
    % 2. Calculate the optimal control move using MPC
    %u = mpcmove(mpc_obj, mpc_state, y_true, r);
    u = mpcmove(mpc_obj, mpc_state, y_true, r_target);
    
    % 3. Simulate the NON-LINEAR plant for one time step
    ode_func = @(t_ode, x_ode) f_handle_3u(x_ode, u);
    
    [T_ode, X_ode] = ode45(ode_func, [t, t + Ts], x_true);
    
    % 4. Update the true state and time
    x_true = X_ode(end, :)';
    t = T_ode(end);
    
    % 5. Log data
    T_log(k+1, 1) = t;
    X_log(k+1, :) = x_true';
    U_log(k+1, :) = u';
end
disp('Simulation complete. Starting animation...');



% --- Animation Setup ---
close all; % Close any existing figures
fig_handle = figure('Visible', 'off'); % Create an invisible figure
ax = gca;
axis([-3 3 -1.5 1.5]);
grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('MPC Control of Non-Linear DIPC System');

cart_width = 0.5;
cart_height = 0.25;

% Create graphics objects
x_cart = X_log(1, 1);
theta1 = X_log(1, 2);
theta2 = X_log(1, 3);
x_m1 = x_cart + l1 * sin(theta1);
y_m1 = l1 * cos(theta1);
x_m2 = x_m1 + l2 * sin(theta2);
y_m2 = y_m1 + l2 * cos(theta2);
y_cart = 0;

cart = rectangle(ax, 'Position', [x_cart - cart_width/2, y_cart - cart_height, cart_width, cart_height], 'FaceColor', [0.5 0.5 0.5]);
bar1 = line(ax, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1], 'LineWidth', 3, 'Color', 'b');
mass1 = line(ax, 'XData', x_m1, 'YData', y_m1, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
bar2 = line(ax, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2], 'LineWidth', 3, 'Color', 'b');
mass2 = line(ax, 'XData', x_m2, 'YData', y_m2, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
time_text = text(ax, -2.8, 1.3, 'Time: 0.00s', 'FontSize', 14);

% --- Video Setup ---
vidObj = VideoWriter(video_filename, 'MPEG-4');
vidObj.FrameRate = 1/Ts;
open(vidObj);

% --- Animation Loop ---
animation_skip = 2; 
for k = 1:animation_skip:length(T_log)
    x_vec = X_log(k, :)'; 
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
    set(time_text, 'String', sprintf('Time: %.2fs', T_log(k)));


    frame = getframe(gcf);
    writeVideo(vidObj, frame);
end
set(fig_handle, 'Visible', 'on');
close(gcf);
close(vidObj);
disp(['Animation video file saved as: ', video_filename]);


end