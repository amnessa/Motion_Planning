% main.m
clear all; close all; clc;

% Configuration parameters
world_radius = 10;
world_center = [0; 0];  % Changed to column vector
start_pos = [-8; -8];   % Changed to column vector
goal_pos = [8; 8];      % Changed to column vector

% Define obstacles: {center_x, center_y, radius}
obstacles = [
    2, 2, 1.5;
    -3, 1, 1.2;
    0, -2, 1.0;
    4, -3, 1.3;
    -2, 4, 1.1
];

% Convert obstacle centers to column vectors for consistency
obstacles_formatted = cell(size(obstacles, 1), 1);
for i = 1:size(obstacles, 1)
    obstacles_formatted{i} = struct('center', obstacles(i,1:2)', 'radius', obstacles(i,3));
end

% Run simulations with different parameters
% Potential Fields
k_att = 1.0;  % Attractive potential gain
k_rep = 100.0;  % Repulsive potential gain
rho_0 = 2.0;  % Influence radius

% Navigation Function
kappa = 10.0;  % Tuning parameter

% Run both planners
results_potential = run_potential_field(start_pos, goal_pos, obstacles_formatted, world_center, world_radius, k_att, k_rep, rho_0);
results_navigation = run_navigation_function(start_pos, goal_pos, obstacles_formatted, world_center, world_radius, kappa);

% Visualize results
visualize_results(results_potential, results_navigation, obstacles, world_center, world_radius, start_pos, goal_pos);

% potential_fields.m
function grad = potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0)
    % Ensure q is a column vector
    q = q(:);
    goal = goal(:);
    world_center = world_center(:);
    
    % Attractive potential gradient
    grad_att = k_att * (q - goal);
    
    % Repulsive potential gradient from obstacles
    grad_rep = zeros(size(q));
    
    % Obstacle repulsion
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        obstacle_radius = obstacles{i}.radius;
        
        % Vector from obstacle to robot
        diff = q - obstacle_center;
        dist = norm(diff);
        
        % If within influence radius
        if dist <= rho_0
            grad_rep = grad_rep + k_rep * (1/dist - 1/rho_0) * ...
                      (1/dist^2) * (diff/dist);
        end
    end
    
    % World boundary repulsion
    diff_boundary = q - world_center;
    dist_boundary = world_radius - norm(diff_boundary);
    if dist_boundary <= rho_0
        grad_rep = grad_rep + k_rep * (1/dist_boundary - 1/rho_0) * ...
                  (1/dist_boundary^2) * (-diff_boundary/norm(diff_boundary));
    end
    
    % Total gradient (ensure column vector)
    grad = grad_att + grad_rep;
end

% navigation_function.m
function [grad] = navigation_function(q, goal, obstacles, world_center, world_radius, kappa)
    % Ensure vectors are column vectors
    q = q(:);
    goal = goal(:);
    world_center = world_center(:);
    
    % Compute gamma (goal function)
    gamma = sum((q - goal).^2);
    
    % Compute beta (obstacle function)
    beta_0 = world_radius^2 - sum((q - world_center).^2);  % Boundary
    beta = beta_0;
    
    % Multiply beta by obstacle functions
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        obstacle_radius = obstacles{i}.radius;
        beta = beta * (sum((q - obstacle_center).^2) - obstacle_radius^2);
    end
    
    % Gradient computation
    grad_gamma = 2 * (q - goal);
    
    % Gradient of beta
    grad_beta = -2 * (q - world_center) * beta_0;  % Boundary gradient
    
    % Obstacle gradients
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        obstacle_radius = obstacles{i}.radius;
        
        beta_i = sum((q - obstacle_center).^2) - obstacle_radius^2;
        grad_beta_i = 2 * (q - obstacle_center);
        
        grad_beta = grad_beta * beta_i + beta * grad_beta_i;
    end
    
    % Complete gradient
    numerator = kappa * gamma^(kappa-1) * beta * grad_gamma - gamma^kappa * grad_beta;
    denominator = (gamma^kappa + beta)^2;
    grad = numerator ./ denominator;
end

% run_potential_field.m
function results = run_potential_field(start, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0)
    % ODE45 setup
    tspan = [0 50];  % Maximum simulation time
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    
    % Define ODE function
    odefun = @(t,q) -potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0);
    
    % Run integration
    [t, q] = ode45(odefun, tspan, start, options);
    
    % Store results
    results.time = t;
    results.path = q;
    results.type = 'Potential Fields';
end

% run_navigation_function.m
function results = run_navigation_function(start, goal, obstacles, world_center, world_radius, kappa)
    % ODE45 setup
    tspan = [0 50];  % Maximum simulation time
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    
    % Define ODE function
    odefun = @(t,q) -navigation_function(q, goal, obstacles, world_center, world_radius, kappa);
    
    % Run integration
    [t, q] = ode45(odefun, tspan, start, options);
    
    % Store results
    results.time = t;
    results.path = q;
    results.type = 'Navigation Function';
end

% visualize_results.m
function visualize_results(results_potential, results_navigation, obstacles, world_center, world_radius, start_pos, goal_pos)
    % Create figure
    figure('Position', [100 100 1200 500]);
    
    % Plot paths
    subplot(1,2,1);
    hold on;
    
    % Draw world boundary
    th = 0:pi/50:2*pi;
    x_bound = world_center(1) + world_radius * cos(th);
    y_bound = world_center(2) + world_radius * sin(th);
    plot(x_bound, y_bound, 'k-', 'LineWidth', 2);
    
    % Draw obstacles
    for i = 1:size(obstacles, 1)
        x_obs = obstacles(i,1) + obstacles(i,3) * cos(th);
        y_obs = obstacles(i,2) + obstacles(i,3) * sin(th);
        fill(x_obs, y_obs, 'r', 'FaceAlpha', 0.3);
    end
    
    % Draw paths
    plot(results_potential.path(:,1), results_potential.path(:,2), 'b-', 'LineWidth', 2);
    plot(results_navigation.path(:,1), results_navigation.path(:,2), 'g-', 'LineWidth', 2);
    
    % Draw start and goal
    plot(start_pos(1), start_pos(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
    plot(goal_pos(1), goal_pos(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    
    grid on;
    axis equal;
    title('Path Comparison');
    legend('World Boundary', 'Obstacles', 'Potential Fields', 'Navigation Function', 'Start', 'Goal');
    
    % Plot convergence
    subplot(1,2,2);
    hold on;
    
    % Compute distances to goal
    dist_pot = sqrt(sum((results_potential.path - goal_pos').^2, 2));
    dist_nav = sqrt(sum((results_navigation.path - goal_pos').^2, 2));
    
    plot(results_potential.time, dist_pot, 'b-', 'LineWidth', 2);
    plot(results_navigation.time, dist_nav, 'g-', 'LineWidth', 2);
    
    grid on;
    title('Distance to Goal vs Time');
    xlabel('Time');
    ylabel('Distance to Goal');
    legend('Potential Fields', 'Navigation Function');
end