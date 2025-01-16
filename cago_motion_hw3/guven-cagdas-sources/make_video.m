function make_video(q_init, q_goal, O, r, path, type)

    % Plot the obstacles
    n_links = length(q_init) - 2;
    figure
    %axis([-n_links*r n_links*5 -n_links*r n_links*r])
    axis(30);
    for i=1:size(O,2)
        Cpatch=O{1,i};
        patch(Cpatch(1,:),Cpatch(2,:),'yellow')
    end
    hold on
    
    % Find initial point of the end effector tip in workspace
    base_pos = q_init(1:2);
    prev_node = base_pos;
    for i=1:n_links
        theta = mod(sum(q_init(3:i+2)), 2*pi);
        next_node = prev_node + [r*cos(theta); r*sin(theta)];
        seg2(:,:,i) = [prev_node next_node];
        prev_node = next_node;
    end
    p_init = next_node;
    
    % Find goal point of the end effector tip in workspace
    base_pos = q_goal(1:2);
    prev_node = base_pos;
    for i=1:n_links
        theta = mod(sum(q_goal(3:i+2)), 2*pi);
        next_node = prev_node + [r*cos(theta); r*sin(theta)];
        seg2(:,:,i) = [prev_node next_node];
        prev_node = next_node;
    end
    p_goal = next_node;
    
    % Plot initial and goal point of end effector in workspace
    plot(p_init(1),p_init(2),'s','MarkerFaceColor','red')
    hold on
    plot(p_goal(1),p_goal(2),'d','MarkerFaceColor','green')
    
    % Initialize video
    myVideo = VideoWriter(strcat(type,'_', int2str(n_links), 'link')); % Open video file with name
    myVideo.FrameRate = 10;  % Set video frame rate
    open(myVideo)
    
    % Initialize base path plot
    base_path_x = [];
    base_path_y = [];

    l(1:n_links) = 0; % Create l to store the robot at each configuration
    hold on
    for i=1:size(path,2)-1
        q_n = path(:,i+1); % Next configuration
        q = path(:,i); % Current configuration
        for t=0:0.05:1 % 21 steps for linear interpolation
            p = (1-t)*q + t*q_n; % Linear interpolation from current to next configuration
            base_pos = p(1:2);

            % Store base position
            base_path_x = [base_path_x, base_pos(1)];
            base_path_y = [base_path_y, base_pos(2)];

            % Plot paths
            plot(base_path_x, base_path_y, 'b-', 'LineWidth', 2);
            
            prev_node = base_pos;
            for j=1:n_links % Loop to plot each link in the manipulator
                theta = mod(sum(p(3:j+2)), 2*pi);
                next_node = prev_node + [r*cos(theta); r*sin(theta)];
                seg2= [prev_node next_node];
                prev_node = next_node;
                l(j) = line(seg2(1,1:2), seg2(2,1:2)); % Plot each link in this configuration
                
            end
            pause(.05)
            frame = getframe(gcf); % Get frame
            writeVideo(myVideo, frame);
            if i == size(path,2)-1 && t==1 % Break if we reach end of path and end of interpolation
                break
            end
            delete(l) % Delete the line
        end
    end
    close(myVideo)
    end
    