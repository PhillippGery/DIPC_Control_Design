%% Entering the initial data

clc;
clear;



theta = 30, %input('Initial angular displacement of the pendulum = ');
theta = theta * pi / 180; % convert from degrees to radians

tfinal = 50; %input('Final time tfinal = ');

data_init = [0 -2]'; % pendulum length
R = [cos(theta) -sin(theta); sin(theta) cos(theta)]; % rotation matrix
data = R * data_init; % initial pendulum position

bar = line('xdata', [0 data(1)], 'ydata', [0 data(2)], 'linewidth', 3);
mass = line('xdata', data(1), 'ydata', data(2), 'marker', 'o', ...
    'markersize', 15, 'markerfacecolor', 'r');
hinge = line('xdata', 0, 'ydata', 0, 'marker', 'o', 'markersize', 7);

axis([-3 3 -3 0.5]);
grid on;
set(gca, 'fontsize', 14);
set(gca, 'dataaspectratio', [1 1 1]);
box on;


%Init Conditions
dt = 0.05; % step-size for solving differential equations can be arbitrarily selected
t = 0; % initial time
thetadot = 0; % initial angular speed





% Set up the video writer object
vidObj = VideoWriter(fullfile(pwd, 'myPendulumVideo.mp4'), 'MPEG-4'); 
vidObj.FrameRate = 30; % Set a frame rate (e.g., 30 fps)
open(vidObj);

% Simulation loop
for i = 1:floor(tfinal / dt)
    t = t + dt;
    
    theta = theta + thetadot * dt;
    thetadot = thetadot - 5 * sin(theta) * dt - 0.001 * thetadot; % friction term
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    data_new = R * data_init;
    
    % Change the property values of the bar and hinge objects
    set(bar, 'xdata', [0 data_new(1)], 'ydata', [0 data_new(2)]);
    set(mass, 'xdata', data_new(1), 'ydata', data_new(2));
    set(hinge, 'xdata', 0, 'ydata', 0);
    
    drawnow;
    
    % Get the current frame and write it to the video file
    currentFrame = getframe(gcf); % Get the current figure frame
    writeVideo(vidObj, currentFrame);
end

% Close the video writer object
close(vidObj);


return



