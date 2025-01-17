function path = RRT_star(q_init, q_goal, O, r, epsilon_0, radius, beta, gamma, min_step, max_iter)
    % RRT* with adaptive step size and bi-objective cost function
    % q_init, q_goal: Initial and goal configurations
    % O: Obstacles
    % r: Link length
    % epsilon_0: Base step size
    % radius: Rewiring radius
    % beta: Weight factor for path length and manipulability
    % gamma: Scaling factor for adaptive step size
    % min_step: Minimum allowable step size
    % max_iter: Maximum iterations to find a solution

    % Initialize variables
    n_links = length(q_init);
    children = q_init; % Initialize tree
    parents = 0; % Root node has no parent
    costs = 0; % Cost to reach each node
    solved = false; % Stop condition

    iter = 0; % Iteration counter
    while iter < max_iter
        iter = iter + 1;

        % Sample a random configuration
        q_rand = 2 * pi * rand(n_links, 1);

        % Find the nearest node in the tree
        q_near = nearest_node(children, q_rand);

        % Compute adaptive step size
        dq = adaptive_dq(q_near, q_goal, epsilon_0, gamma, min_step, r);

        % Generate a new configuration
        q_new = q_near + dq * (q_rand - q_near) / norm(q_rand - q_near);

        % Check for collisions
        if point_collides(q_new, r, O) || path_collides(q_near, q_new, r, O)
            continue;
        end

        % Find the index of q_near in the children array
        near_index = find(all(children == q_near, 1), 1);

        % Compute cost for q_new
        manipulability_cost_new = manipulability_cost(q_new, r);
        cost_new = beta * (costs(near_index) + norm(q_new - q_near)) + ...
                    (1 - beta) * manipulability_cost_new;

        % Add q_new to the tree
        children = [children, q_new];
        parents = [parents, near_index];
        costs = [costs, cost_new];

        % Rewire the tree
        neighbors = find_neighbors(children, q_new, radius);
        for i = 1:size(neighbors, 2)
            neighbor = neighbors(:, i);
            neighbor_index = find(all(children == neighbor, 1), 1);
            temp_cost = beta * (cost_new + norm(q_new - neighbor)) + ...
                        (1 - beta) * manipulability_cost(neighbor, r);
            if temp_cost < costs(neighbor_index) && ~path_collides(q_new, neighbor, r, O)
                costs(neighbor_index) = temp_cost;
                parents(neighbor_index) = size(children, 2); % Rewire parent
            end
        end

        % Check if goal is reached
        if ~solved && ~path_collides(q_new, q_goal, r, O) && norm(q_new - q_goal) <= 3
            fprintf('Goal connected at node %d\n', size(children, 2));
            solved = true;

            % Ask user for additional iterations
            while true
                user_input = input('Path to goal found. Enter additional iterations (0 to stop): ', 's');
                additional_iter = str2double(user_input);
                if isnan(additional_iter) || additional_iter < 0
                    fprintf('Invalid input. Please enter a non-negative number.\n');
                elseif additional_iter == 0
                    fprintf('Stopping optimization. Finalizing path...\n');
                    break;
                else
                    fprintf('Continuing optimization for %d more iterations...\n', additional_iter);
                    max_iter = iter + additional_iter; % Extend iterations
                    break;
                end
            end

            if additional_iter == 0
                break; % Exit the main loop
            end
        end

        % Optional: Debugging output every 100 iterations
        if mod(iter, 100) == 0
            fprintf('Iteration %d: Nodes = %d\n', iter, size(children, 2));
        end
    end

    % Reconstruct the path
    if solved
        path = build_path(children, parents, q_init, q_goal);
        path = fliplr(path); % Reverse path for start-to-goal order
    else
        warning('RRT* failed to find a path within the maximum number of iterations.');
        path = [];
    end
end
    