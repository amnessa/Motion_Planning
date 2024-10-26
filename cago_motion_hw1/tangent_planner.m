function [x_path, y_path] = tangent_planner(qstart, qgoal)
    % Initialize the path with the start position
    x_path = qstart(1);
    y_path = qstart(2);
    
    % Define tolerance for reaching the goal
    goal_tolerance = 0.1;
    
    % Set the current position to the start
    current_position = qstart;
    
    % Set a maximum step size and reduced sensor step angles
    max_step_size = 0.2;  % Increase step size for faster movement
    sensor_step_angle = pi / 15;  % Fewer angle steps to reduce computations
    
    % Move towards the goal
    while norm(current_position - qgoal) > goal_tolerance
        % Calculate the direction to the goal
        goal_direction = atan2(qgoal(2) - current_position(2), qgoal(1) - current_position(1));
        
        % Get distance to the closest obstacle in the goal direction
        distance_to_obstacle = read_sensor(goal_direction, current_position);
        
        if distance_to_obstacle > norm(current_position - qgoal)
            % If no obstacle is in the way, move directly towards the goal
            step_size = min(max_step_size, norm(qgoal - current_position));
            current_position = current_position + step_size * [cos(goal_direction), sin(goal_direction)];
        else
            % If an obstacle is detected, follow the obstacle while checking if the goal is visible
            [current_position, is_goal_visible] = follow_obstacle_lightweight(current_position, qgoal, goal_direction, sensor_step_angle);
            
            % If the goal becomes visible during boundary following, resume heading towards the goal
            if is_goal_visible
                continue;  % Exit boundary following and go back to goal tracking
            end
        end
        
        % Store the new position
        x_path(end + 1) = current_position(1);
        y_path(end + 1) = current_position(2);
    end
end

%% Optimized Obstacle Following for Tangent Bug
function [new_position, is_goal_visible] = follow_obstacle_lightweight(current_position, qgoal, goal_direction, sensor_step_angle)
    global sensor_range infinity;
    
    % Step size and angles for tracing the obstacle
    step_size = 0.1;  % Larger step size to move faster along the boundary
    
    % Check if goal is visible from current position
    distance_to_goal = norm(current_position - qgoal);
    
    % If there is no obstacle between the robot and the goal, return
    if read_sensor(goal_direction, current_position) >= distance_to_goal
        is_goal_visible = true;
        new_position = current_position;
        return;
    end
    
    % Follow the obstacle's boundary
    is_goal_visible = false;
    
    % Trace along the obstacle by checking fewer sensor readings in different directions
    for angle_offset = -pi: sensor_step_angle: pi
        sensing_angle = goal_direction + angle_offset;
        distance_to_obstacle = read_sensor(sensing_angle, current_position);
        
        % Move in the direction where the obstacle is closer (along the boundary)
        if distance_to_obstacle < infinity / 2 && distance_to_obstacle < sensor_range
            step_direction = [cos(sensing_angle), sin(sensing_angle)];
            new_position = current_position + step_size * step_direction;
            
            % Check if the goal is visible again after a small step
            if read_sensor(goal_direction, new_position) >= distance_to_goal
                is_goal_visible = true;
            end
            
            return;
        end
    end
    
    % If no valid movement is found, stay in the same position (edge case)
    new_position = current_position;
end
