% potential_fields.m
function grad = potential_fields(q, goal, obstacles, world_center, world_radius, epsilon_q, epsilon_r)
    q = q(:);
    goal = goal(:);
    
    % Calculate attractive gradient
    d_goal = norm(q - goal);
    grad_att = epsilon_q * (q - goal) / d_goal;
    
    % Enhanced repulsive gradient with tangential component
    grad_rep = zeros(size(q));
    
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        Q_min = obstacles{i}.radius;
        Q_inf = 2 * world_radius;
        
        diff_obs = q - obstacle_center;
        d_obs = norm(diff_obs);
        
        if d_obs < Q_inf
            % Normal repulsion
            normal_rep = epsilon_r * (1/d_obs - 1/Q_inf) * (1/d_obs^2) * (diff_obs/d_obs);
            
            % Tangential component (rotated 90 degrees)
            tangent = [-diff_obs(2); diff_obs(1)] / d_obs; % Tangential force
            % note to future calculation causes obstacles to pull when
            % theres big difference btw parameters
            % Combine normal and tangential components
            grad_rep = grad_rep + normal_rep + 1.5 * epsilon_r * tangent; % Amplified tangential force
        end
    end
    
    grad = grad_att + grad_rep;
end
