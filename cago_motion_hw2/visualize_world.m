
% visualize_world.m - Function to visualize the environment and paths
function visualize_world(world_center, world_radius, obstacles, path)
    figure;
    hold on;
    axis equal;
    
    % Draw world boundary
    th = 0:pi/50:2*pi;
    x = world_center(1) + world_radius * cos(th);
    y = world_center(2) + world_radius * sin(th);
    plot(x, y, 'k-', 'LineWidth', 2);
    
    % Draw obstacles
    for i = 1:length(obstacles.center)
        x = obstacles.center{i}(1) + obstacles.radius{i} * cos(th);
        y = obstacles.center{i}(2) + obstacles.radius{i} * sin(th);
        fill(x, y, 'r', 'FaceAlpha', 0.3);
    end
    
    % Draw path
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
    
    grid on;
    xlabel('X');
    ylabel('Y');
    title('Motion Planning Result');
end