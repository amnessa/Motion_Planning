function b = path_collides(q_near, q_new, r, O)
step_collides = zeros(1,21); %array to store boolean of whether or not each step collides
count = 1;

for t=0:0.05:1 %21-step linear interpolation
    p = (1-t)*q_near + t*q_new; %linear interpolation formula
    [b, ~] = point_collides(p, r, O); %check whether p collides

    if b ~= 0 %if the step collides, store this in step_collides and break
        step_collides(count) = 1; 
        continue
    end
    count = count+1;
end

if sum(step_collides) == 0 %if no steps collide return false
    b = false;
else
    b = true; %if any steps collide return true
end

end