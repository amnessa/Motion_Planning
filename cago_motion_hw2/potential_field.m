% potential_field.m - Implementation of potential field approach
function [gradients] = potential_field(q, goal, obstacles, world_center, world_radius)
    % Parameters
    k_att = 1.0;  % Attractive potential gain
    k_rep = 100.0;  % Repulsive potential gain
    rho_0 = 2.0;  % Influence radius of obstacles
    
    % Attractive gradient
    grad_att = k_att * (q - goal);
    
    % Repulsive gradient from obstacles
    grad_rep = zeros(size(q));
    for i = 1:length(obstacles.center)
        dist = norm(q - obstacles.center{i});
        if dist <= rho_0
            grad_rep = grad_rep + k_rep * (1/dist - 1/rho_0) * ...
                      (1/dist^2) * (q - obstacles.center{i});
        end
    end
    
    % Boundary repulsion
    dist_boundary = world_radius - norm(q - world_center);
    if dist_boundary <= rho_0
        grad_rep = grad_rep + k_rep * (1/dist_boundary - 1/rho_0) * ...
                  (1/dist_boundary^2) * (q - world_center);
    end
    
    gradients = grad_att + grad_rep;
end