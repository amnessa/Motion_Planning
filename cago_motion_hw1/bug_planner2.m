function [x_path, y_path] = bug_planner2(qstart, qgoal)
    % Initialize the path with the start position
    x_path = qstart(1);
    y_path = qstart(2);
    
    % Define tolerance for reaching the goal
    goal_tolerance = 0.1;
    
    % Set the current position to the start
    current_position = qstart;
    
    % Move towards the goal in a straight line
    while norm(current_position - qgoal) > goal_tolerance
        % Calculate the direction to the goal
        goal_direction = atan2(qgoal(2) - current_position(2), qgoal(1) - current_position(1));
        
        % Read sensor data in the goal direction
        distance_to_obstacle = read_sensor(goal_direction, current_position);
        
        if distance_to_obstacle < 0.5
            % If an obstacle is detected, follow its boundary
            current_position = follow_boundary(current_position, qgoal, goal_direction);
        else
            % Move towards the goal in a straight line
            step_size = min(0.1, norm(qgoal - current_position));
            current_position = current_position + step_size * [cos(goal_direction), sin(goal_direction)];
        end
        
        % Store the new position
        x_path(end + 1) = current_position(1);
        y_path(end + 1) = current_position(2);
    end
end

function new_position = follow_boundary(current_position, qgoal, goal_direction)
    % Follow the boundary of the obstacle
    % This is a simple implementation that traces along the obstacle’s boundary
    
    % Initialize sensor step angle and step size
    sensor_step_angle = pi / 10;
    step_size = 0.05;
    
    % Rotate until no obstacle is detected in the current direction
    while read_sensor(goal_direction, current_position) < 0.5
        % Step in the direction along the obstacle boundary
        goal_direction = goal_direction + sensor_step_angle;  % Rotate along the boundary
        step_direction = [cos(goal_direction), sin(goal_direction)];
        
        % Take a small step along the boundary
        current_position = current_position + step_size * step_direction;
    end
    
    % Once the boundary is cleared, return the new position
    new_position = current_position;
end
