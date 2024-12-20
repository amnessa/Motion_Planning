function [b, seg] = point_collides(q_rand, r, O)
    n_links = length(q_rand) - 2;
    seg = zeros(2,2,n_links); % Stores location of each segment of the robot in the workspace
    base_pos = q_rand(1:2); % Extract base position
    prev_node = base_pos; % Start from base position
    
    % This loop finds and stores the location of each joint in the workspace
    for i=1:n_links
        theta = mod(sum(q_rand(3:i+2)), 2*pi);
        next_node = prev_node + [r*cos(theta); r*sin(theta)];
        seg(:,:,i) = [prev_node next_node];
        prev_node = next_node;
    end
    
    collides(n_links,size(O,2)) = 0; % Array to store collision of each obstacle with each node
    for j=1:n_links
        for i = 1:size(O,2)
            collides(j,i) = isintersect_linepolygon(seg(:,:,j), O{1, i}); % Check intersection for each segment
        end
    end
    
    b = any(collides(:)); % Return true if any link collides with any obstacle
    end
    