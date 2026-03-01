function [T_log, X_log, X_hat_log, U_log] = run_manual_mpc_sim(f_handle_3u, Ad, Bd, Cd, Kx, Kr, Ld, x_init, h, tfinal, r_target, xe, ue, video_filename)
% This function simulates the non-linear plant, but the controller is a
% manually-calculated discrete-time MPC (Kx, Kr) using states estimated
% by a discrete-time Luenberger observer (Ld).

disp('Starting manual MPC + Observer simulation...');

% --- 1. Simulation Setup ---
t_steps = floor(tfinal / h);
n = size(Ad, 1);
m = size(Bd, 2);
p_horizon = size(Kr, 2) / size(Cd, 1); % Get horizon from gain matrix

% --- 2. Initialize States and Logs ---
t = 0;
x_true = x_init; % The true, non-linear state
x_hat = x_init;  % The observer's estimated state
u = ue;          % The last applied input

% Create the reference deviation vector
r_dev = r_target - (Cd * xe);
r_p_dev = kron(ones(p_horizon, 1), r_dev);

% Initialize logs
T_log = t;
X_log = x_true';
X_hat_log = x_hat';
U_log = u';

% Define physical constants for animation
l1 = 0.5; l2 = 0.75;

% --- 3. Simulation Loop (Discrete Steps) ---
for k = 1:t_steps
    
    % --- Controller and Observer Logic (at time k) ---
    
    % 1. OBSERVER: Update the state estimate using the measurement from the *previous* step
    % We do this first, to get x_hat[k]
    y_true = Cd * x_true;
    y_dev = y_true - (Cd * xe);
    delta_x_hat = x_hat - xe;
    
    % Luenberger Observer Equation (in deviation variables)
    % x_hat_dev[k] = Ad*x_hat_dev[k-1] + Bd*u_dev[k-1] + Ld*(y_dev[k] - C*x_hat_dev[k-1])
    delta_u_prev = u - ue;
    delta_x_hat = Ad*delta_x_hat + Bd*delta_u_prev + Ld*(y_dev - Cd*delta_x_hat);
    
    % Update the absolute estimated state
    x_hat = delta_x_hat + xe;
    
    % 2. CONTROLLER: Calculate new control input u[k]
    % Use the non-augmented MPC law (Eq. 12) on the estimated deviation state
    % delta_u_vector = Kr * r_p_dev - Kx * delta_x_hat
    delta_u_all = Kr * r_p_dev - Kx * delta_x_hat;
    
    % Receding Horizon: We only apply the first control input
    delta_u = delta_u_all(1:m); 
    
    % Convert to absolute input
    u = ue + delta_u;
    
    % 3. PLANT: Simulate the NON-LINEAR plant for one time step
    ode_func = @(t_ode, x_ode) f_handle_3u(x_ode, u);
    [T_ode, X_ode] = ode45(ode_func, [t, t + h], x_true);
    
    % 4. Update true state and time
    x_true = X_ode(end, :)';
    t = T_ode(end);
    
    % 5. Log data
    T_log(k+1, 1) = t;
    X_log(k+1, :) = x_true';
    X_hat_log(k+1, :) = x_hat';
    U_log(k+1, :) = u';
end
disp('Simulation complete. Starting animation...');



% --- 4. Animation Setup (identical to your template) ---
fig_handle = figure('Visible', 'off');
ax = gca;
axis([-3 3 -1.5 1.5]);
grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('Manual MPC  + Luenberger Observer');
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

% --- Video Setup ---
vidObj = VideoWriter(video_filename, 'MPEG-4');
vidObj.FrameRate = 1/h;
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