function neighbors = find_neighbors(tree, q_new, radius)
    % find_neighbors - Finds nodes within a given radius of q_new
    %
    % Inputs:
    %   tree: Array of existing nodes (each column is a node configuration)
    %   q_new: The new node (column vector)
    %   radius: Radius within which to search for neighbors
    %
    % Outputs:
    %   neighbors: Array of neighboring nodes within the radius

    % Compute distances from q_new to all nodes in the tree
    distances = vecnorm(tree - q_new, 2, 1); % Euclidean norm (2-norm)

    % Find nodes within the specified radius
    neighbor_indices = find(distances <= radius);
    neighbors = tree(:, neighbor_indices);
end
