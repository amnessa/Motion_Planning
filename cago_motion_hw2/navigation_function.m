% navigation_function.m - Implementation of navigation function approach
function [phi_grad] = navigation_function(q, goal, obstacles, world_center, world_radius)
    % Parameters
    kappa = 10.0;  % Navigation function gain
    
    % Goal function (gamma)
    gamma = norm(q - goal)^2;
    
    % Obstacle function (beta)
    beta_0 = world_radius^2 - norm(q - world_center)^2;  % Boundary
    beta = beta_0;
    for i = 1:length(obstacles.center)
        beta = beta * (norm(q - obstacles.center{i})^2 - obstacles.radius{i}^2);
    end
    
    % Navigation function
    phi = (gamma^kappa / (gamma^kappa + beta))^(1/kappa);
    
    % Compute gradient (implementation needed)
    phi_grad = zeros(size(q));  % Placeholder
    % TODO: Implement gradient computation
end