% potential_fields.m
function grad = potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0, k_bound, rho_b)
    q = q(:);
    goal = goal(:);
    world_center = world_center(:);
    
    % Attractive potential gradient (quadratic)
    grad_att = k_att * (q - goal);
    
    % Repulsive potential gradient
    grad_rep = zeros(size(q));
    
    % Obstacle repulsion with correct formula
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        obstacle_radius = obstacles{i}.radius;
        
        diff = q - obstacle_center;
        dist = norm(diff);
        
        if dist <= rho_0
            % Proper repulsive gradient formula
            grad_rep = grad_rep + k_rep * (1/dist - 1/rho_0) * ...
                      (1/(dist^2)) * (diff/dist);
        end
    end
    
    % World boundary repulsion with same formula
    diff_boundary = q - world_center;
    dist_boundary = world_radius - norm(diff_boundary);
    if dist_boundary <= rho_b
        grad_rep = grad_rep + k_bound * (1/dist_boundary - 1/rho_b) * ...
                  (1/(dist_boundary^2)) * (-diff_boundary/norm(diff_boundary));
    end
    
    % Total gradient
    grad = grad_att + grad_rep;
end