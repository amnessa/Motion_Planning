function cost = manipulability_cost(q, r)
    q = q(:); % Ensure column vector
    n_links = length(q);
    L = ones(n_links, 1) * r; % Link lengths

    % Precompute cumulative angles for all joints
    cumulative_angles = cumsum(q); % [θ1, θ1+θ2, ..., θ1+...+θn]

    % Initialize Jacobian
    J = zeros(2, n_links);

    for i = 1:n_links
        % For joint i, sum contributions from links i to end
        links = i:n_links;
        J(1, i) = -sum(L(links) .* sin(cumulative_angles(links)));
        J(2, i) = sum(L(links) .* cos(cumulative_angles(links)));
    end

    % Compute manipulability measure
    if rank(J) < 2
        cost = 1e3; % Singular configuration
    else
        manipulability_measure = sqrt(det(J * J'));
        cost = 1 / max(manipulability_measure, 1e-3); % Avoid division by zero
    end
end