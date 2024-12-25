clear all
clc


%% RUN PRM FUNCTION

% q_init and q_goal now include base position
q_init = [-2; 8; 0; pi/4; -pi/4];
q_goal = [20; 27; pi/2; -pi/4; pi/4];

n = 1000; K = 10;
O = {[5 30 30 5; 10 10 12 12], [-5 -5 5 5; 18 20 20 18], [10 12 12 10; 15 15 25 25], [20 22 22 20; 5 5 20 20],[12 12 0 0;25 27 27 25]};

r = 2; % This is the length of each link of the robot

% Define workspace limits for the base position
workspace_limits = [30; 30]; % Example limits for x and y

path = PRM(q_init, q_goal, n, K, O, r, workspace_limits); % Pass workspace_limits to PRM
make_video(q_init, q_goal, O, r, path, 'PRM'); % Function to use the path to make the video of robot in workspace
