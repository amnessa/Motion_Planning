clear all
clc


%% RUN PRM FUNCTION

% q_init and q_goal now include base position
q_init = [-5; -5; 0; 2*pi-.1; 0.1];
q_goal = [18; 25; pi/2; pi/4; 2*pi - pi/4];
n = 1000; K = 10;
O = {};
for i = 1:30
    center = [2+randi(25); 2+randi(25)];
    radius = 2;
    theta = linspace(0, 2*pi, 50);
    O{end+1} = [center(1) + radius*cos(theta); center(2) + radius*sin(theta)];
end

r = 2; % This is the length of each link of the robot

% Define workspace limits for the base position
workspace_limits = [30; 30]; % Example limits for x and y

path = PRM(q_init, q_goal, n, K, O, r, workspace_limits); % Pass workspace_limits to PRM
make_video(q_init, q_goal, O, r, path, 'PRM'); % Function to use the path to make the video of robot in workspace
