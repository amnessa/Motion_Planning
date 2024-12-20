function path = build_path(children, parents, q_init, q_goal)
current_child = children(:,end); %find current child as the last node in children array
path = q_goal; %build a path backwards starting from the goal

while norm(path(:,end)-q_init) > 0.0001 %while the last member of path is not q_init
    for i=1:size(children,2) %for each node in the children array
        if children(:,i) == current_child %if this node is the current child
            current_parent = parents(:,i); %find the parent of this child as the corresponding point in the parent array
            
            %if current parent is q_init, add current child and q_init to
            %path and exit
            if current_parent == q_init
                path = [path current_child q_init];
                return
            end

            path = [path current_child]; %add current child to the path
            current_child = current_parent; %update cyrrent child to the current parent
        end
    end
end
end