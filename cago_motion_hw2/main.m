% main.m
clear all; close all; clc;

% Configuration parameters
world_radius = 10;
world_center = [0, 0];
start_pos = [-8, -8];
goal_pos = [8, 8];

% Define obstacles: {center_x, center_y, radius}
obstacles = [
    2, 2, 1.5;
    -3, 1, 1.2;
    0, -2, 1.0;
    4, -3, 1.3;
    -2, 4, 1.1
];

% Run simulations with different parameters
% Potential Fields
k_att = 1.0;  % Attractive potential gain
k_rep = 100.0;  % Repulsive potential gain
rho_0 = 2.0;  % Influence radius

% Navigation Function
kappa = 10.0;  % Tuning parameter

% Run both planners
results_potential = run_potential_field(start_pos, goal_pos, obstacles, world_center, world_radius, k_att, k_rep, rho_0);
results_navigation = run_navigation_function(start_pos, goal_pos, obstacles, world_center, world_radius, kappa);

% Visualize results
visualize_results(results_potential, results_navigation, obstacles, world_center, world_radius, start_pos, goal_pos);
