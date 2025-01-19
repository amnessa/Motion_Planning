function dq = adaptive_dq(q_near, q_goal, epsilon_0, gamma, min_step, r)
    % adaptive_dq - Compute an adaptive step size based on alignment with manipulability and distance
    %
    % Inputs:
    %   q_near: Current configuration (column vector)
    %   q_goal: Goal configuration (column vector)
    %   epsilon_0: Base step size (scalar)
    %   gamma: Scaling factor for step size adjustment (scalar)
    %   min_step: Minimum allowable step size (scalar)
    %   r: Link length (used for manipulability measure)
    %
    % Output:
    %   dq: Adaptive step size (scalar)

    % Compute manipulability gradient term
    omega = manipulability_cost(q_near, r);
    omega_star = 0.5; % Threshold manipulability
    eta_omega = 0.02; % Strength of manipulability field
    if omega <= omega_star
        grad_omega = eta_omega * (1 / omega - 1 / omega_star) * (1 / omega^2);
    else
        grad_omega = 0;
    end

    % Compute distance gradient term
    dist = norm(q_goal - q_near);
    d_star = 0.01; % Threshold distance
    eta_dist = 0.005; % Strength of distance field
    if dist <= d_star
        grad_dist = eta_dist * (1 / dist - 1 / d_star) * (1 / dist^2);
    else
        grad_dist = 0;
    end

    % Total gradient
    grad_total = grad_omega + grad_dist;

    % Compute adaptive step size
    dq = epsilon_0 - gamma * grad_total;

    % Enforce minimum step size
    dq = max(dq, min_step);
end
