function f = hueristic(g, n_init, n_goal)
%part A hueristic function, finds f, min number of nodes from n_init to n_goal

%convert weighted adjacency matrix into an adjacency matrix
g(g==Inf)=0;
g(g~=0)=1;
N = size(g, 1);

%if n_init and n_goal are not neighbors
if g(n_init, n_goal) == 0  
    %deal with case where n_init = n_goal
    if n_init == n_goal
        f = 0;
        return
    end
    
    %iterate through each node in the graph because this is the maximum
    %possible number of steps between two nodes
    for i=1:N
        
        %use property where the minimum power of adjacency matrix to get
        %n_init, n_goal to be nonzero is the minimum number of nodes
        %between n_init and n_goal
        check = g^i;
        if check(n_init, n_goal) ~= 0
            f = i;
            return
        end
    end

%else if n_init and n_goal are neighbors
else
    f = 1;
end

end