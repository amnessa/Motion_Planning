function [value, isterminal, direction] = collision_event(t, q, obstacles)
    min_dist = inf;
    for i = 1:length(obstacles)
        obstacle_center = obstacles{i}.center;
        obstacle_radius = obstacles{i}.radius;
        dist = norm(q - obstacle_center) - obstacle_radius;
        min_dist = min(min_dist, dist);
    end
    value = min_dist - 0.5;  % Stop if closer than 0.5 units to any obstacle
    isterminal = 1;
    direction = 0;
end