function b = path_collides(q_near, q_new, r, O)
step_collides = zeros(1,21); % Array to store boolean of whether or not each step collides
count = 1;

for t=0:0.05:1 % 21-step linear interpolation
    p = (1-t)*q_near + t*q_new; % Linear interpolation formula
    [b, ~] = point_collides(p, r, O); % Check whether p collides

    if b % If the step collides, store this in step_collides
        step_collides(count) = 1; 
    end
    count = count+1;
end

b = any(step_collides); % Return true if any steps collide
end
    