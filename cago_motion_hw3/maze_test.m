clear all
clc


%% RUN PRM FUNCTION

% q_init and q_goal now include base position
q_init = [3; 8; 0; pi/4; -pi/4];
q_goal = [30; 8; pi/2; -pi/4; pi/4];

n = 600; K = 20;
O = {[5 35 35 5; 10 10 12 12], [5 7 7 5; 10 30 30 10], [10 12 12 10; 15 15 25 25], [20 22 22 20; 5 5 20 20]};

r = 2; % This is the length of each link of the robot

% Define workspace limits for the base position
workspace_limits = [30; 30]; % Example limits for x and y

path = PRM(q_init, q_goal, n, K, O, r, workspace_limits); % Pass workspace_limits to PRM
make_video(q_init, q_goal, O, r, path, 'PRM'); % Function to use the path to make the video of robot in workspace
