function path = RRT_star(q_init, q_goal, O, r, dq, radius, beta)
    % RRT* with bi-objective cost function: path length and manipulability
    % q_init, q_goal: initial and goal configurations
    % O: obstacles
    % r: link length
    % dq: step size
    % radius: rewiring radius
    % beta: weight factor balancing path length and manipulability

    n_links = length(q_init);
    children = q_init; % Initialize child nodes in the tree
    parents = q_init; % Initialize parent nodes in the tree
    costs = 0; % Initialize cost to reach each node
    solved = false; % Condition to exit while loop

    while ~solved
        % Sample a random configuration
        q_rand = 2 * pi * rand(n_links, 1);

        % Find the nearest node already in the tree
        q_near = nearest_node(children, q_rand);
        q_new = q_near + dq * (q_rand - q_near); % Take a step towards the random point

        % Check for collisions with obstacles
        if point_collides(q_new, r, O)
            continue;
        end

        % Check collision along the path from q_near to q_new
        if path_collides(q_near, q_new, r, O)
            continue;
        end

        % Find the index of q_near in the children array
        near_index = find(all(children == q_near, 1), 1);
        if isempty(near_index)
            continue;
        end

        % Calculate the manipulability cost for q_new
        manipulability_cost_new = manipulability_cost(q_new, r);

        % Calculate the bi-objective cost
        cost_new = beta * (costs(near_index) + norm(q_new - q_near)) + ...
                   (1 - beta) * manipulability_cost_new;

        % Find neighbors within the rewiring radius
        neighbors = find_neighbors(children, q_new, radius);

        % Find the best parent from neighbors
        best_parent = q_near;
        best_cost = cost_new;

        for i = 1:size(neighbors, 2)
            neighbor = neighbors(:, i);
            neighbor_index = find(all(children == neighbor, 1), 1);
            if isempty(neighbor_index)
                continue;
            end

            % Calculate the bi-objective cost for this neighbor
            manipulability_cost_neighbor = manipulability_cost(neighbor, r);
            temp_cost = beta * (costs(neighbor_index) + norm(q_new - neighbor)) + ...
                        (1 - beta) * manipulability_cost_neighbor;

            if ~path_collides(neighbor, q_new, r, O) && temp_cost < best_cost
                best_cost = temp_cost;
                best_parent = neighbor;
            end
        end

        % Update parent and cost for q_new
        parents = [parents best_parent];
        children = [children q_new];
        costs = [costs best_cost];

        % Rewire the tree
        for i = 1:size(neighbors, 2)
            neighbor = neighbors(:, i);
            neighbor_index = find(all(children == neighbor, 1), 1);
            if isempty(neighbor_index)
                continue;
            end

            % Calculate the bi-objective cost for rewiring
            manipulability_cost_neighbor = manipulability_cost(neighbor, r);
            temp_cost = beta * (best_cost + norm(q_new - neighbor)) + ...
                        (1 - beta) * manipulability_cost_neighbor;

            if temp_cost < costs(neighbor_index) && ~path_collides(q_new, neighbor, r, O)
                % Update the cost and parent
                costs(neighbor_index) = temp_cost;
                parents(:, neighbor_index) = q_new;
            end
        end

        % Check if goal is reached
        if ~path_collides(q_new, q_goal, r, O) && norm(q_new - q_goal) <= 3
            solved = true;
        end
    end

    % Rebuild path from q_goal to q_init
    path = build_path(children, parents, q_init, q_goal);
    path = fliplr(path); % Reverse path to go from init to goal
end

function neighbors = find_neighbors(tree, q_new, radius)
    % Vectorized operation to find neighbors
    distances = vecnorm(tree - q_new, 2, 1);
    neighbors = tree(:, distances <= radius);
end
