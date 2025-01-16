function b = isintersect_linepolygon(S,Q)
q0 = Q(:,1);
Q = [Q q0];
n = size(Q,2);
p0 = S(:,1); p1 = S(:,2);

if p0 == p1 %if given a single point instead of a line segment
    b = inpolygon(p0(1), p0(2), Q(1,:), Q(2,:)); %use inpolygon to test whether point is in shape
else
    %otherwise follow given pseudocode to find whether line segment
    %intersects with the polygon
    te = 0; tl = 1;
    ds = p1 - p0;
    for i=1:n-1
        n_i = [0 1; -1 0]*(Q(:,i+1) - Q(:,i)); %find the outward normal vector as a -pi/2 rotation from each edge of the polygon
        
        % follows given pseudocode exactly to determine intersection
        N = -dot(p0 - Q(:,i),n_i);
        D = dot(ds,n_i);
        if D == 0
            if N<0
                b = false;
                return
            end
        end

        t = N/D;
        if D < 0 
            te = max(te, t);
            if te > tl 
                b = false;
                return
            end
        elseif D > 0
            tl = min(tl, t);
            if tl<te 
                b = false;
                return
            end
        end
    end

    if te <= tl
        b = true;
        return
    else
        b = false;
        return
    end
end
end