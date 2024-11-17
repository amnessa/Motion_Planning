function [value, isterminal, direction] = collision_event(t,q,obstacles)
    min_dist = inf;
    for i = 1:length(obstacles)
        dist = norm(q - obstacles{i}.center) - obstacles{i}.radius;
        min_dist = min(min_dist, dist);
    end
    value = min_dist - 0.5;  % Stop if closer than 0.5 units to any obstacle
    isterminal = 1;
    direction = 0;
end