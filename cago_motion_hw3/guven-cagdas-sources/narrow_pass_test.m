clear all
clc


%% RUN PRM FUNCTION

% q_init and q_goal now include base position
q_init = [-5; 15; 0; pi/4; -pi/4];
q_goal = [20; 18; pi/2; -pi/4; pi/4];
n = 1000; K = 10;
O = {[5 5 10 10 ;-5 17 17 -5], [5 5 10 10;20 35 35 20 ]};

r = 2; % This is the length of each link of the robot

% Define workspace limits for the base position
workspace_limits = [30; 30]; % Example limits for x and y

path = PRM(q_init, q_goal, n, K, O, r, workspace_limits); % Pass workspace_limits to PRM
make_video(q_init, q_goal, O, r, path, 'PRM'); % Function to use the path to make the video of robot in workspace
