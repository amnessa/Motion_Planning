function dq = adaptive_dq(q_nearest, q_goal, r, manipulability_cost_fn, epsilon_0, gamma, min_step)
    % adaptive_dq - Computes the adaptive step size for RRT* based on manipulability and distance to goal
    % 
    % Inputs:
    %   q_nearest: Current configuration (column vector)
    %   q_goal: Goal configuration (column vector)
    %   r: Link length (scalar)
    %   manipulability_cost_fn: Handle to the manipulability cost function
    %   epsilon_0: Base step size (scalar)
    %   gamma: Scaling factor for adaptive step size (scalar)
    %   min_step: Minimum allowable step size (scalar)
    %
    % Outputs:
    %   dq: Adaptive step size (scalar)

    % Compute manipulability cost for the current configuration
    U_manipulability = manipulability_cost_fn(q_nearest, r);
    
    % Compute distance-to-goal potential
    U_distance = norm(q_goal - q_nearest);
    
    % Combine potentials into a repulsive field
    U = U_manipulability + U_distance;
    
    % Compute gradient of the potential field numerically
    grad_U = zeros(size(q_nearest));
    delta = 1e-5; % Small perturbation for numerical differentiation
    for i = 1:length(q_nearest)
        % Perturb each joint angle
        q_perturbed = q_nearest;
        q_perturbed(i) = q_perturbed(i) + delta;
        
        % Compute partial derivative
        U_perturbed = manipulability_cost_fn(q_perturbed, r) + norm(q_goal - q_perturbed);
        grad_U(i) = (U_perturbed - U) / delta;
    end

    % Compute adaptive step size
    dq = epsilon_0 - gamma * norm(grad_U);
    
    % Enforce minimum step size
    dq = max(dq, min_step);
end
