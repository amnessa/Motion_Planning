function [b, seg] = point_collides(q_rand, r, O)
n_links = length(q_rand);
seg =zeros(2,2,n_links); %stores location of each segment of the robot in the workspace
prev_node = [0;0]; %this is where the base of the robot is fixed

%this loop finds and store the location of each joint in the workspace
for i=1:n_links
    theta = mod(sum(q_rand(1:i)), 2*pi);
    next_node = prev_node + [r*cos(theta); r*sin(theta)];
    seg(:,:,i) = [prev_node next_node];
    prev_node = next_node;
end

collides(n_links,size(O,2)) = 0; %array to store collision of each obstacle with each node
for j=1:n_links
    for i = 1:size(O,2)
        collides(j,i) = isintersect_linepolygon(seg(:,:,j), O{1, i}); %use isintersect_linepolygon to check intersection for each segment
    end
end

if sum(collides) == 0 %if each link does not collide with any obstacle, return false
    b = false; 
else %else return true
    b = true;
end
end