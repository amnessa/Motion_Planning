% main.m - Main script to run experiments and generate results
clear all; close all; clc;

% Configuration parameters
world_radius = 10;  % Radius of the circular world boundary
world_center = [0, 0];  % Center of the world
start_pos = [-5, -5];  % Starting position
goal_pos = [5, 5];   % Goal position

% Example obstacle configuration
obstacles = struct('center', {[2,2], [-3,1], [0,-2]}, ...
                  'radius', {1, 1.5, 1});

% Run both planners and compare results
results_potential = run_potential_field(start_pos, goal_pos, obstacles, world_center, world_radius);
results_navigation = run_navigation_function(start_pos, goal_pos, obstacles, world_center, world_radius);

% Visualize and compare results
compare_results(results_potential, results_navigation);