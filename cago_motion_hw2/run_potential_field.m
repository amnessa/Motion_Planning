% run_potential_field.m
function results = run_potential_field(start, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0, k_bound, rho_b)
    % ODE45 setup
    tspan = linspace(0, 30, 1000);  % More time points for smoother path
    options = odeset('RelTol', 1e-2, 'AbsTol', 1e-2, 'MaxStep', 0.01, 'InitialStep', 0.005, 'Events', @(t,q) collision_event(t,q,obstacles));
   
    % Define ODE function with closure over parameters
    odefun = @(t,q) potential_fields_wrapper(t, q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0, k_bound, rho_b);
    
    % Run integration
    [t, q] = ode45(odefun, tspan, start, options);
    
    % Store results
    results.time = t;
    results.path = q;
    results.type = 'Potential Fields';
end

function dq = potential_fields_wrapper(t, q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0, k_bound, rho_b)
    if norm(q - goal) < 0.1
        dq = [0; 0];
    else
        dq = -potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0, k_bound, rho_b);
    end
end