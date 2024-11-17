
% visualize_results.m
function visualize_results(results_potential, results_navigation, obstacles, world_center, world_radius, start_pos, goal_pos)
    % Create figure
    figure('Position', [100 100 1200 500]);
    
    % Plot paths
    subplot(1,2,1);
    hold on;
    
    % Draw world boundary
    th = 0:pi/50:2*pi;
    x_bound = world_center(1) + world_radius * cos(th);
    y_bound = world_center(2) + world_radius * sin(th);
    plot(x_bound, y_bound, 'k-', 'LineWidth', 2);
    
    % Draw obstacles
    for i = 1:size(obstacles, 1)
        x_obs = obstacles(i,1) + obstacles(i,3) * cos(th);
        y_obs = obstacles(i,2) + obstacles(i,3) * sin(th);
        fill(x_obs, y_obs, 'r', 'FaceAlpha', 0.3);
    end
    
    % Draw paths
    plot(results_potential.path(:,1), results_potential.path(:,2), 'b-', 'LineWidth', 2);
    plot(results_navigation.path(:,1), results_navigation.path(:,2), 'g-', 'LineWidth', 2);
    
    % Draw start and goal
    plot(start_pos(1), start_pos(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
    plot(goal_pos(1), goal_pos(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    
    grid on;
    axis equal;
    title('Path Comparison');
    legend('World Boundary', 'Obstacles', 'Potential Fields', 'Navigation Function', 'Start', 'Goal');
    
    % Plot convergence
    subplot(1,2,2);
    hold on;
    
    % Update the distance calculation line
    dist_pot = sqrt(sum((results_potential.path - repmat(goal_pos', size(results_potential.path, 1), 1)).^2, 2));
    dist_nav = sqrt(sum((results_navigation.path - repmat(goal_pos', size(results_navigation.path, 1), 1)).^2, 2));

    
    plot(results_potential.time, dist_pot, 'b-', 'LineWidth', 2);
    plot(results_navigation.time, dist_nav, 'g-', 'LineWidth', 2);
    
    grid on;
    title('Distance to Goal vs Time');
    xlabel('Time');
    ylabel('Distance to Goal');
    legend('Potential Fields', 'Navigation Function');
end