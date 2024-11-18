% main.m
clear all; close all; clc;

% World and obstacle parameters
world_radius = 60;
world_center = [0; 0];
start_pos = [-35; -35];
goal_pos = [30; 40];

% Define obstacles with spacing
obstacles = [
    10, 10, 7.0;    % [x, y, radius]
    -15, 5, 6.0;
    %0, -10, 5.0;
    %20, -15, 6.5;
    %-10, 20, 5.5
];

% Convert obstacle centers to column vectors for consistency
obstacles_formatted = cell(size(obstacles, 1), 1);
for i = 1:size(obstacles, 1)
    obstacles_formatted{i} = struct('center', obstacles(i,1:2)', 'radius', obstacles(i,3));
end

% Influence radii
obstacle_influence_radius = world_radius * 0.1;  % 10% of world radius

% Enhanced parameters
epsilon_q = 80.0;    % goal attraction parameter
epsilon_r = 10.0;     % obstacle repulsion parameter

% Parameter for navigation function
kappa = 10.0;  % Tuning parameter

% Run planner
results_potential = run_potential_field(start_pos, goal_pos, obstacles_formatted, world_center, world_radius, epsilon_q, epsilon_r);
results_navigation = run_navigation_function(start_pos, goal_pos, obstacles_formatted, world_center, world_radius, kappa);

% Visualize results
visualize_results(results_potential, results_navigation, obstacles, world_center, world_radius, start_pos, goal_pos);
