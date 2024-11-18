function dq = potential_fields_with_stop( q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0)
    if norm(q - goal) < 0.1
        dq = [0; 0];
    else
        dq = -potential_fields(q, goal, obstacles, world_center, world_radius, k_att, k_rep, rho_0);
    end
end