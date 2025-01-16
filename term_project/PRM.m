function path = PRM(q_init, q_goal, n, K, O, r)
n_links = length(q_init);
V = q_init; %store the vertices
G(1:n,1:n) = Inf; %initialize adjacency graph
segs =zeros(2,2,n_links,n); %create 4D array to store the location of each link joint in each frame

while size(V, 2) < n-1 %while less than n nodes in Cfree
    for i=1:n_links
        q_rand(i) = 2*pi*rand;
    end
    [b, seg] = point_collides(q_rand, r, O); %find whether point is collision free and get the location in the workspace of the segments in the robot
    if ~b %if random point in Cfree is collision-free
        segs(:,:,:,size(V,2)) = seg; %add the location of the segments for each collision-free point
        V = [V q_rand']; %then add the point to the nodes
    end
end
V = [V q_goal]; %add goal point to nodes

%add goal node location of segements into V
prev_node = [0;0];
for i=1:n_links
    theta = mod(sum(q_goal(1:i)), 2*pi);
    next_node = prev_node + [r*cos(theta); r*sin(theta)];
    segs(:,:,i,n) = [prev_node next_node];
end


for m = 1:size(V, 2) %for each vertex in the graph
    q = V(:,m); 
    
    %calculate the distance to all of the other nodes in the graph
    dists(1:size(V,2)) = 0;
    for i = 1:size(V,2)
        dists(i) = norm(q - V(:,i));
    end
    [~, n_idx] = mink(dists(dists~= 0), K); %find the K nearest neighbors to q

    for x = n_idx %for each neighbor
        q_n = V(:,x);

        %if the path does not collide to the neighbor
        if ~path_collides(q, q_n, r, O)
            G(m, x) = dists(x); %then add the distance to the adjacency matrix
            G(x, m) = dists(x);
        end
    end

    for i=1:size(G,2) %set diagonal of the adjacency matrix to zero
        G(i,i) = 0;
    end
end

%use Astar to find the path from the adjacency matrix
disp('finding path')
points = Astar(G, 1, n, @hueristic);
path = V(:,points);
end