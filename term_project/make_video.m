function make_video(q_init, q_goal, O, r, path, type)

%plot the obstacles
n_links = length(q_init);
figure
axis([-n_links*r n_links*5 -n_links*r n_links*r])
for i=1:size(O,2)
    Cpatch=O{1,i};
    patch(Cpatch(1,:),Cpatch(2,:),'yellow')
end
hold on

%find initial point of the end effector tip in workspace
prev_node = [0;0];
for i=1:n_links
    theta = mod(sum(q_init(1:i)), 2*pi);
    next_node = prev_node + [r*cos(theta); r*sin(theta)];
    seg2(:,:,i) = [prev_node next_node];
    prev_node = next_node;
end
p_init = next_node;

%find goal point of the end effector tip in workspace
prev_node = [0;0];
for i=1:n_links
    theta = mod(sum(q_goal(1:i)), 2*pi);
    next_node = prev_node + [r*cos(theta); r*sin(theta)];
    seg2(:,:,i) = [prev_node next_node];
    prev_node = next_node;
end
p_goal = next_node;

%plot inital and goal point of end effector in workspace
plot(p_init(1),p_init(2),'s','MarkerFaceColor','red')
hold on
plot(p_goal(1),p_goal(2),'d','MarkerFaceColor','green')


myVideo = VideoWriter(strcat(type,'_', int2str(n_links), 'link')); %open video file with name
myVideo.FrameRate = 10;  %set video frame rate
open(myVideo)
l(1:n_links) = 0; %create l to store the robot at each configuration
hold on
for i=1:size(path,2)-1
    q_n = path(:,i+1); %next configuration
    q = path(:,i); %current configuration
    for t=0:0.05:1 %21 steps for linear interpolation
        p = (1-t)*q + t*q_n; %linear interpolation from current to next cofiguration
        prev_node = [0;0];
        for j=1:n_links %loop to plot each link in the manipulator
            theta = mod(sum(p(1:j)), 2*pi);
            next_node = prev_node + [r*cos(theta); r*sin(theta)];
            seg2= [prev_node next_node];
            prev_node = next_node;
            l(j) = line(seg2(1,1:2), seg2(2,1:2)); %plot each link in this configuration
        end
        pause(.05)
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
        if i == size(path,2)-1 && t==1 %break if we reach end of path and end of interpolation
            break
        end
        delete(l) %delete the line
    end
end
close(myVideo)
end