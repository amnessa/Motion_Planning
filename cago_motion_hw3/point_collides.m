function [collisionDetected, segments] = point_collides(q_rand, link_length, obstacles)
    % Number of links
    n_links = length(q_rand) - 2; % Exclude base position
    segments = zeros(2, 2, n_links); % Stores line segments of the robot links
    base_pos = q_rand(1:2); % Extract base position
    prev_node = base_pos; % Start with the base position
    collisionDetected = false; % Initialize collision flag
    resolution = 10; % Number of intermediate points for collision checking

    % Compute the position of each joint and link segment
    for i = 1:n_links
        theta = mod(sum(q_rand(3:i+2)), 2*pi); % Cumulative joint angle
        next_node = prev_node + [link_length * cos(theta); link_length * sin(theta)];
        segments(:, :, i) = [prev_node, next_node]; % Store segment as [start, end]
        % Generate intermediate points for finer collision checking
        interp_points = linspace(0, 1, resolution); 
        for t = interp_points
            intermediate_point = (1 - t) * prev_node + t * next_node;
            % Check for collision with each obstacle
            for j = 1:numel(obstacles)
                if inpolygon(intermediate_point(1), intermediate_point(2), obstacles{j}(1, :), obstacles{j}(2, :))
                    collisionDetected = true;
                    return;
                end
            end
        end
        prev_node = next_node;
    end
end
