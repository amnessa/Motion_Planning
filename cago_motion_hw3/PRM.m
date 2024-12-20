function path = PRM(q_init, q_goal, n, K, O, r, workspace_limits)
    % Assume q_init and q_goal now include base position, e.g., [x_base; y_base; joint_angles]
    
    n_links = length(q_init) - 2; % Adjust for base position
    V = q_init; % Store the vertices
    G(1:n,1:n) = Inf; % Initialize adjacency graph
    segs = zeros(2,2,n_links,n); % Create 4D array to store the location of each link joint in each frame
    
    while size(V, 2) < n-1 % While less than n nodes in Cfree
        % Generate random base position and joint angles
        q_rand = [rand(2, 1) .* workspace_limits; 2*pi*rand(n_links, 1)];
        
        [b, seg] = point_collides(q_rand, r, O); % Check collision
        if ~b % If collision-free
            segs(:,:,:,size(V,2)) = seg; % Store segment locations
            V = [V q_rand]; % Add to vertices
        end
    end
    V = [V q_goal]; % Add goal point to nodes
    
    % Add goal node location of segments into V
    % (Update this section to include base position)
    
    for m = 1:size(V, 2) % For each vertex in the graph
        q = V(:,m); 
        
        % Calculate the distance to all other nodes in the graph
        dists = vecnorm(V - q, 2, 1);
        [~, n_idx] = mink(dists(dists~= 0), K); % Find the K nearest neighbors
    
        for x = n_idx % For each neighbor
            q_n = V(:,x);
    
            % If the path does not collide to the neighbor
            if ~path_collides(q, q_n, r, O)
                G(m, x) = dists(x); % Add distance to adjacency matrix
                G(x, m) = dists(x);
            end
        end
    
        for i=1:size(G,2) % Set diagonal of adjacency matrix to zero
            G(i,i) = 0;
        end
    end
    
    % Use A* to find the path from the adjacency matrix
    disp('finding path')
    points = Astar(G, 1, n, @hueristic);
    path = V(:,points);
    end
    