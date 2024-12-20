function q_near = nearest_node(children, q_rand)
min_dist = Inf;
n = size(children,2);

for i=1:n %for all nodes in the children array
    dist = norm(q_rand - children(:,i)); %find distance of each node in the children array to the given input node
    if dist < min_dist %if their distance is less than current min
        min_dist = dist; %update the minimum distance
        q_near = children(:,i); %and store this as the closest node
    end
end
end