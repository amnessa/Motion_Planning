% run_potential_field.m - Function to run potential field planner
function results = run_potential_field(start, goal, obstacles, world_center, world_radius)
    % Parameters
    dt = 0.01;  % Time step
    max_steps = 1000;  % Maximum number of steps
    
    % Initialize
    path = zeros(max_steps, 2);
    path(1,:) = start;
    
    % ODE45 implementation
    tspan = [0 10];
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    [t, q] = ode45(@(t,q) -potential_field(q, goal, obstacles, world_center, world_radius), ...
                    tspan, start, options);
    
    results = struct('path', q, 'time', t);
end