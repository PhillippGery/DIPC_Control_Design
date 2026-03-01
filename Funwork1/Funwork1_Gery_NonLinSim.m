%% DIPC NON-LINEAR SIMULATION

close all;

% =========================================================================
%  1. SYMBOLIC DERIVATION (Run Once)
% =========================================================================
fprintf('Deriving non-linear equations of motion symbolically...\n');

% Define symbolic variables
syms M m1 m2 l1 l2 g
syms theta1 theta2 x_pos theta1_dot theta2_dot x_dot
syms theta1_ddot theta2_ddot x_ddot
syms u

% Define the state vector (for matlabFunction variables)
state_vector = [x_pos; theta1; theta2; x_dot; theta1_dot; theta2_dot];

% Define the non-linear Equations of Motion (EOMs)
eq1 = (M+m1+m2)*x_ddot + (m1+m2)*l1*theta1_ddot*cos(theta1) + m2*l2*theta2_ddot*cos(theta2) - (m1+m2)*l1*theta1_dot^2*sin(theta1) - m2*l2*theta2_dot^2*sin(theta2) - u == 0;
eq2 = (m1+m2)*l1^2*theta1_ddot + (m1+m2)*l1*x_ddot*cos(theta1) + m2*l1*l2*theta2_ddot*cos(theta1-theta2) + m2*l1*l2*theta2_dot^2*sin(theta1-theta2) - (m1+m2)*g*l1*sin(theta1) == 0;
eq3 = m2*l2^2*theta2_ddot + m2*l2*x_ddot*cos(theta2) + m2*l1*l2*theta1_ddot*cos(theta1-theta2) - m2*l1*l2*theta1_dot^2*sin(theta1-theta2) - m2*g*l2*sin(theta2) == 0;

% Solve for the accelerations
eqddot = solve([eq1, eq2, eq3], [x_ddot, theta1_ddot, theta2_ddot]);

% Build the state-derivative vector f(x,u)
f_symbolic = [x_dot; theta1_dot; theta2_dot; eqddot.x_ddot; eqddot.theta1_ddot; eqddot.theta2_ddot];

% Substitute numerical parameter values into the symbolic expression
M_val = 1.5; m1_val = 0.5; m2_val = 0.75;
l1_val = 0.5; l2_val = 0.75; g_val = 9.81;
f_symbolic = subs(f_symbolic, {M, m1, m2, l1, l2, g}, {M_val, m1_val, m2_val, l1_val, l2_val, g_val});

% *** Convert the symbolic expression into an optimized MATLAB function handle ***
f_handle = matlabFunction(f_symbolic, 'Vars', {state_vector, u});

fprintf('Non-linear function handle created.\n\n');

% =========================================================================
%  2. NUMERICAL SIMULATION
% =========================================================================
% --- Simulation Parameters ---
tfinal = 10;
dt = 0.05;
x_min = -2.5;
x_max = 2.5;
b_friction = 0; % Friction coefficient

% --- Initial Conditions ---
x_vec = [0; 0.01; 0.02; 0; 0; 0]; % [x; theta1; theta2; x_dot; ...]

% --- Graphics and Video Initialization (same as before) ---
cart_width = 0.5; 
cart_height = 0.25;
figure; 
ax = gca;
axis([-3 3 -2.5 2.5]); grid on;
set(ax, 'DataAspectRatio', [1 1 1]);
title('NON-LINEAR Double Inverted Pendulum Simulation');
x_cart = x_vec(1); y_cart = 0;
x_m1 = x_cart + l1_val * sin(x_vec(2)); y_m1 = l1_val * cos(x_vec(2));
x_m2 = x_m1 + l2_val * sin(x_vec(3)); y_m2 = y_m1 + l2_val * cos(x_vec(3));
cart = rectangle(ax, 'Position', [x_cart - cart_width/2, y_cart - cart_height/2, cart_width, cart_height], 'FaceColor', [0.5 0.5 0.5]);
xline(ax, x_min, 'k--', 'LineWidth', 2); xline(ax, x_max, 'k--', 'LineWidth', 2);
bar1 = line(ax, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1], 'LineWidth', 3, 'Color', 'b');
mass1 = line(ax, 'XData', x_m1, 'YData', y_m1, 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
bar2 = line(ax, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2], 'LineWidth', 3, 'Color', 'r');
mass2 = line(ax, 'XData', x_m2, 'YData', y_m2, 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
time_text = text(ax, -2.8, 2.2, 'Time: 0.00s', 'FontSize', 14);
vidObj = VideoWriter('dipc_nonlinear_simulation.mp4', 'MPEG-4');
vidObj.FrameRate = 1/dt;
open(vidObj);

% --- Simulation Loop ---
for t = 0:dt:tfinal
    
    % Define the input (uncontrolled system)
    u_input = 0;
    
    % *** Calculate state derivative using the NON-LINEAR function handle ***
    x_dot = f_handle(x_vec, u_input);
   
    
    % Update state using Forward Euler integration
    x_vec = x_vec + x_dot * dt;
    
    % Boundary checking (same as before)
    if x_vec(1) >= x_max;
        x_vec(1) = x_max;
        if x_vec(4) >= 0
            x_vec(4) = 0; 
        end
    elseif x_vec(1) <= x_min;
        x_vec(1) = x_min;
        if x_vec(4) <= 0
            x_vec(4) = 0;
        end
    end
    
    % Update Graphics
    x_cart = x_vec(1);
    theta1 = x_vec(2);
    theta2 = x_vec(3);
    x_m1 = x_cart + l1_val * sin(theta1);
    y_m1 = l1_val * cos(theta1);
    x_m2 = x_m1 + l2_val * sin(theta2);
    y_m2 = y_m1 + l2_val * cos(theta2);

    set(cart, 'Position', [x_cart - cart_width/2, y_cart - cart_height/2, cart_width, cart_height]);
    set(bar1, 'XData', [x_cart, x_m1], 'YData', [y_cart, y_m1]);
    set(mass1, 'XData', x_m1, 'YData', y_m1);
    set(bar2, 'XData', [x_m1, x_m2], 'YData', [y_m1, y_m2]);
    set(mass2, 'XData', x_m2, 'YData', y_m2);
    set(time_text, 'String', sprintf('Time: %.2fs', t));
    drawnow;
    frame = getframe(gcf);
    writeVideo(vidObj, frame);
end

% --- Cleanup ---
close(vidObj);
fprintf('Non-linear simulation complete. Video saved as dipc_nonlinear_simulation.mp4\n');