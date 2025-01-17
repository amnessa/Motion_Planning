function path = build_path(children, parents, q_init, q_goal)
    current_child = children(:, end); % Start from the last node
    path = q_goal; % Initialize path from the goal

    while ~isequal(path(:, end), q_init) % While not at the start
        % Locate the parent of the current child
        child_index = find(all(children == current_child, 1), 1);

        if isempty(child_index)
            error('Invalid child index: %d. Check tree consistency.', child_index);
        end

        parent_index = parents(child_index); % Retrieve parent index

        % Handle the root node
        if parent_index == 0
            break; % Stop at the root node
        end

        if parent_index > size(children, 2) || parent_index < 1
            error('Invalid parent index: %d for child index: %d', parent_index, child_index);
        end

        current_parent = children(:, parent_index); % Get the parent node
        path = [path, current_child]; % Append current child to the path
        current_child = current_parent; % Move to the parent
    end

    path = [path, q_init]; % Append the start node
end
