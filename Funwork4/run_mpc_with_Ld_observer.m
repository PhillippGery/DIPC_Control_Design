function [T_log, X_log, U_log, X_hat_log] = run_mpc_with_Ld_observer(mpc_obj, f_handle_3u, Ad, Bd, Cd, Ld, C_u3, x_init, h, tfinal, r_target, xe, ue, video_filename)
% This function simulates the NON-LINEAR plant, controlled by an MPC,
% using a user-supplied Luenberger observer (Ld) for state estimation.
%
% Inputs:
%   mpc_obj      - The MPC controller object
%   f_handle_3u  - Non-linear plant dynamics
%   Ad, Bd, Cd   - Discrete-time linear model matrices (for observer)
%   Ld           - Discrete-time Luenberger observer gain (6x3)
%   C_u3         - Continuous-time (real) output matrix
%   x_init       - Initial true state (6x1)
%   h            - Sampling time
%   tfinal       - Total simulation time
%   r_target     - Output reference (3x1)
%   xe, ue       - Equilibrium state (6x1) and input (3x1)
%   video_filename - Name for the output .mp4 file
%
% Outputs:
%   T_log        - Time vector (Nx1)
%   X_log        - True state history (Nx6)
%   U_log        - Input history (Nx3)
%   X_hat_log    - Estimated state history (Nx6)

disp('Starting NON-LINEAR simulation with MPC and external Luenberger Observer...');
% --- Simulation Setup ---
% --- Simulation Setup ---
t_steps = floor(tfinal / h); % Number of simulation steps

% --- FIX: Manually initialize the observer state and input ---
x_hat = x_init; % Initialize estimated state (e.g., at true state)
u = ue;         % Initialize input to equilibrium
x_true = x_init;
t = 0;

% Initialize logs
T_log = t;
X_log = x_true';
U_log = u';
X_hat_log = x_hat';

% --- FIX: Disable the MPC's internal observer ---
% This tells mpcmove to accept a numerical state vector
% instead of an mpcstate object.
%mpc_obj.StateObserver = []; 

% Define physical constants for animation
l1 = 0.5; l2 = 0.75;

% --- Simulation Loop (Discrete Steps) ---
for k = 1:t_steps
    
    % 1. Measure the "true" plant output from the current state
    y_true = C_u3 * x_true;

    % 2. Manually update the Luenberger Observer
    % x_hat[k] = Ad*x_hat[k-1] + Bd*u[k-1] + Ld*(y[k] - Cd*x_hat[k-1])
    y_hat = Cd * x_hat;
    x_hat = Ad*x_hat + Bd*u + Ld*(y_true - y_hat);
    
    % 3. Calculate control move using the EXTERNAL state estimate
    % We pass the deviation state (x_hat - xe) as the 'x' argument.
    delta_x_hat = x_hat - xe;
    u = mpcmove(mpc_obj, delta_x_hat, y_true, r_target);
    
    % 4. Simulate the NON-LINEAR plant for one time step
    ode_func = @(t_ode, x_ode) f_handle_3u(x_ode, u);    
    [T_ode, X_ode] = ode45(ode_func, [t, t + h], x_true);
    
    x_true = X_ode(end, :)';
    t = T_ode(end);
    
    % 5. Log data
    T_log(k+1, 1) = t;
    X_log(k+1, :) = x_true';
    U_log(k+1, :) = u';
    X_hat_log(k+1, :) = x_hat';
end
disp('Simulation complete. Starting animation...');

% --- Animation Setup (identical to your template) ---
fig_handle = figure('Visible', 'off'); 
ax = gca;
axis([-3 3 -1.5 1.5]);
grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('MPC Control with Luenberger Observer');
cart_width = 0.5;
cart_height = 0.25;
y_cart = 0;
x_cart = X_log(1, 1);
theta1 = X_log(1, 2);
theta2 = X_log(1, 3);
x_m1 = x_cart + l1 * sin(theta1);
y_m1 = l1 * cos(theta1);
x_m2 = x_m1 + l2 * sin(theta2);
y_m2 = y_m1 + l2 * cos(theta2);
cart = rectangle(ax, 'Position', [x_cart - cart_width/2, y_cart - cart_height, cart_width, cart_height], 'FaceColor', [0.5 0.5 0.5]);
bar1 = line(ax, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1], 'LineWidth', 3, 'Color', 'b');
mass1 = line(ax, 'XData', x_m1, 'YData', y_m1, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
bar2 = line(ax, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2], 'LineWidth', 3, 'Color', 'b');
mass2 = line(ax, 'XData', x_m2, 'YData', y_m2, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
time_text = text(ax, -2.8, 1.3, 'Time: 0.00s', 'FontSize', 14);

% --- Video Setup (identical) ---
vidObj = VideoWriter(video_filename, 'MPEG-4');
vidObj.FrameRate = 1/h;
open(vidObj);

% --- Animation Loop (identical) ---
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