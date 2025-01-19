function cost = manipulability_cost(q, r)
    % Manipulability cost calculation for a 3+ link manipulator
    % q: joint angles (column vector)
    % r: link length (scalar, uniform for all links)

    % Ensure q is a column vector
    q = q(:);

    n_links = length(q);
    L = ones(1, n_links) * r; % Uniform link lengths

    % Initialize Jacobian matrix
    J = zeros(2, n_links);

    % Compute the Jacobian matrix
    for i = 1:n_links
        % Extract relevant joint angles as a row vector
        q_segment = q(1:i).'; % Convert to row vector

        % Use only the first i link lengths
        L_segment = L(1:i);

        % Compute Jacobian terms
        J(1, i) = -sum(L_segment .* sin(cumsum(q_segment)));
        J(2, i) = sum(L_segment .* cos(cumsum(q_segment)));
    end

    % Singular Value Decomposition
    [~, S, ~] = svd(J);

    % Extract singular values
    singular_values = diag(S);

    % Check for rank-deficient Jacobian (singular configuration)
    if rank(J) < 2
        % If rank is less than 2, assign a very high cost
        cost = 1e3;
        return;
    end

    % Smallest singular value indicates closeness to singularity
    manipulability_measure = sqrt(det(J * J.'));

    % Cost is inversely proportional to manipulability measure
    if manipulability_measure > 1e-3
        cost = 1 / manipulability_measure; % Avoid division by near-zero
    else
        cost = 1e3; % Assign a high cost near singularities
    end
end