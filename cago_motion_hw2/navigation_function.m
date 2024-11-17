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