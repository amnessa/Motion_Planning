function [collisionDetected, segments] = point_collides(q_rand, link_length, obstacles)
    % Detects collisions between a manipulator and obstacles
    % q_rand: Joint angles (column vector, no base position included)
    % link_length: Uniform link length (scalar)
    % obstacles: Cell array of polygonal obstacles (vertices in [x, y] format)
    
    % Number of links
    n_links = length(q_rand);
    
    % Initialize storage for link segments
    segments = zeros(2, 2, n_links); % [start_x, end_x; start_y, end_y] for each link
    collisionDetected = false; % Initialize collision flag
    resolution = 10; % Number of intermediate points for finer collision checking

    % Base position is fixed at the origin
    prev_node = [0; 0]; % Start at the origin

    % Compute the position of each joint and link segment
    for i = 1:n_links
        % Compute the cumulative joint angle
        theta = mod(sum(q_rand(1:i)), 2*pi); % Accumulate joint angles up to link i
        
        % Compute the end position of the current link
        next_node = prev_node + [link_length * cos(theta); link_length * sin(theta)];
        
        % Store the segment as [start, end]
        segments(:, :, i) = [prev_node, next_node];
        
        % Generate intermediate points for finer collision checking
        interp_points = linspace(0, 1, resolution);
        for t = interp_points
            % Interpolate between start and end points
            intermediate_point = (1 - t) * prev_node + t * next_node;
            
            % Check for collision with each obstacle
            for j = 1:numel(obstacles)
                if inpolygon(intermediate_point(1), intermediate_point(2), obstacles{j}(1, :), obstacles{j}(2, :))
                    collisionDetected = true;
                    return;
                end
            end
        end
        
        % Update the previous node for the next link
        prev_node = next_node;
    end
end
