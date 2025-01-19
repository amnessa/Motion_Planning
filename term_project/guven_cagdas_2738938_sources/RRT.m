function path = RRT(q_init, q_goal, O, r, dq)
n_links = length(q_init);
children = q_init; %setup up array where I will store all child nodes in the tree
parents = q_init; %set up array where I will store all parent nodes in the tree
solved = false; %while loop condition boolean

while ~solved
    for i=1:n_links
        q_rand(i) = 2*pi*rand;
    end
    q_near = nearest_node(children, q_rand); %find nearest point already in the tree 
    q_new = q_near + dq*(q_rand' - q_near); %take step of size dq from the nearest point in tree to the random point
    [b1, ~] = point_collides(q_new, r, O); %check whether q_new collides with obstacles

    if b1 %if q_new collides with obstacles, path will not be clear so break and start over
        continue;
    end

    b = path_collides(q_near, q_new, r, O); %check collision in 21 steps from q_near to q_new
    if ~b
        parents = [parents q_near]; %add q_near to the parent array
        children = [children q_new]; %add q_new to the children array
        if ~path_collides(q_new, q_goal, r, O) && norm(q_new - q_goal) <= 3 %if we are close to the goal and no collision between where we are now and goal, then break
            solved = true;
        end
    end
end

path = build_path(children, parents, q_init, q_goal); %rebuild path
path = fliplr(path); %flip path so it's from init to goal
end