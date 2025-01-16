function path = Astar(gr, n_init, n_goal, hueristic)
    %initialize open and closed set    
    N = size(gr, 1);
    o = [];
    c = [];
    o(1) = n_init;
    
    %set up g, f, and prev
    g(1:N) = Inf;
    g(n_init) = 0;
    f(1:N) = Inf;
    f(n_init) = hueristic(gr, n_init, n_goal);
    prev(1:N) = nan;
    
    %until the open set is empty
    while isempty(o) == 0
        %find hueristic function values for all open nodes
        fn = [];
        fn(1:length(o)) = 0;
        for i = 1:length(o)
            fn(i) = hueristic(gr, o(i), n_goal);
        end
        
        %choose n_best as node with the minimum f value
        [~, idx] = min(fn);
        n_best = o(idx);

        %remove n_best from o and move it to c
        o(idx) = [];
        c = horzcat(c, n_best);
        
        %if we reach n_goal, then break out of loop
        if n_best == n_goal
            break;
        end
        
        %find all neighbors of c
        neighbors = find(gr(n_best, :) ~= Inf);
        neighbors = neighbors(neighbors ~= n_best);
        
        %for each neighbor of c
        for x = neighbors
            if ismember(c, x) == 1 %if the neighbor is closed, move on to next neighbor
                continue
            end

            %calculate g_temp
            g_temp = g(n_best) + gr(n_best, x);
            
            %if the neighbor is not in the open set or closed set, then add
            %it to open set
            if (ismember(x, o) == 0) && ismember(x, c) == 0
                o = horzcat(o, x);
            elseif g_temp >= g(x)
                continue
            end
            
            %update prev, g, and x
            prev(x) = n_best;
            g(x) = g_temp;
            f(x) = g(x) + hueristic(gr, x, n_goal);
        end
    end
    
    %reconstruct the path from prev
    path = [];
    last = prev(n_goal);

    %until n_init is on the path
    while ismember(n_init, path) == 0
        path = horzcat(path, last); %apend the previous node to the path
        last = prev(last); %update the current node to the previous
    end
    path = fliplr(path); %flip the path so it is from n_init -> n_goal
    
    %if the last value on the path is not n_goal, add it
    if path(end) ~= n_goal
        path = horzcat(path, n_goal);
    end
end