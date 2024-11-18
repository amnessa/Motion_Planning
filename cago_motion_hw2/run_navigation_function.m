% run_navigation_function.m
function results = run_navigation_function(start, goal, obstacles, world_center, world_radius, kappa)
    % ODE45 setup
    tspan = [0 1000];
    options = odeset('RelTol', 1e-2, ...
                'AbsTol', 1e-2, ...
                'MaxStep', 0.01, ...
                'InitialStep', 0.005, ...
                'Events', @(t,q) collision_event(t,q,obstacles));    
    % Define ODE function
    odefun = @(t,q) -navigation_function(q, goal, obstacles, world_center, world_radius, kappa);
    
    % Run integration
    [t, q] = ode45(odefun, tspan, start, options);
    
    % Store results
    results.time = t;
    results.path = q;
    results.type = 'Navigation Function';
end
