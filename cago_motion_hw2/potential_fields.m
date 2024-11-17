% potential_fields.m
function grad = potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0)
    % Attractive potential gradient
    grad_att = k_att * (q - goal);
    
    % Repulsive potential gradient from obstacles
    grad_rep = zeros(size(q));
    
    % Obstacle repulsion
    for i = 1:size(obstacles, 1)
        obstacle_center = obstacles(i, 1:2);
        obstacle_radius = obstacles(i, 3);
        
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
    
    % Total gradient
    grad = grad_att + grad_rep;
end
