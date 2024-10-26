function [x_path, y_path] = bug_planner(qstart, qgoal, use_tangent_bug)
    % Initialize the path with the start position
    
    % memory preallocation for speed
    x_path = zeros(1,1000000);
    y_path = zeros(1,1000000);

    x_path = qstart(1);
    y_path = qstart(2);
    
    % Define tolerance for reaching the goal
    goal_tolerance = 0.1;
    
    % Set the current position to the start
    current_position = qstart;
    
    % Set flag to choose between Bug 2 and Tangent Bug boundary following
    if nargin < 3
        use_tangent_bug = false;  % Default is to use Bug 2 boundary following
    end

    
    
    % Move towards the goal in a straight line
    while norm(current_position - qgoal) > goal_tolerance
        % Calculate the direction to the goal
        goal_direction = atan2(qgoal(2) - current_position(2), qgoal(1) - current_position(1));
        
        % Read sensor data in the goal direction
        distance_to_obstacle = read_sensor(goal_direction, current_position);
        
        if distance_to_obstacle > norm(current_position - qgoal)
            % If no obstacle is in the way, move directly towards the goal
            step_size = min(goal_tolerance, norm(qgoal - current_position));
            current_position = current_position + step_size * [cos(goal_direction), sin(goal_direction)];
        else
            % If an obstacle is detected, follow the boundary
            if use_tangent_bug
                % Use Tangent Bug boundary following
                current_position = follow_boundary_tangentbug(current_position, qgoal);
            else
                % Use traditional Bug 2 boundary following
                current_position = follow_boundary_bug2(current_position, qgoal);
            end
        end
        
        % Store the new position
        x_path(end + 1) = current_position(1);
        y_path(end + 1) = current_position(2);
    end
end

%% Bug 2 Boundary Following
function new_position = follow_boundary_bug2(current_position, qgoal)
    % Traditional Bug 2 boundary following:
    % The robot will move along the obstacle boundary until it can
    % leave the boundary and head directly towards the goal again.
    
    % Initialize sensor step angle and step size
    sensor_step_angle = pi / 10;
    step_size = 0.05;
    
    % Rotate until no obstacle is detected in the current direction
    goal_direction = atan2(qgoal(2) - current_position(2), qgoal(1) - current_position(1));
    
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

%% Tangent Bug Boundary Following
function new_position = follow_boundary_tangentbug(current_position, qgoal)
    global sensor_range infinity;
    
    % Step size and angles for tracing the obstacle
    step_size = 0.05;  % Step size along the obstacle boundary
    clearance_distance = 0.1;  % Desired constant distance from the obstacle
    
    sensor_step_angle = pi / 20;  % Angle step for checking around the obstacle boundary
    
    % Check if goal is visible from current position
    goal_direction = atan2(qgoal(2) - current_position(2), qgoal(1) - current_position(1));
    distance_to_goal = norm(current_position - qgoal);
    
    % If the goal is visible directly, move towards it
    if read_sensor(goal_direction, current_position) >= distance_to_goal
        new_position = current_position;
        return;
    end
    
    % Otherwise, follow the boundary while maintaining a constant distance
    for angle_offset = -pi: sensor_step_angle: pi
        sensing_angle = goal_direction + angle_offset;
        distance_to_obstacle = read_sensor(sensing_angle, current_position);
        
        % Move in the direction where the obstacle is closer and maintain constant distance
        if distance_to_obstacle < infinity / 2 && distance_to_obstacle < sensor_range
            % Calculate the perpendicular direction to maintain constant distance
            angle_to_obstacle = sensing_angle + pi / 2;  % Perpendicular to obstacle surface
            perpendicular_direction = [cos(angle_to_obstacle), sin(angle_to_obstacle)];
            
            % Adjust position to maintain clearance distance from the obstacle
            adjustment = (clearance_distance - distance_to_obstacle) * perpendicular_direction;
            step_direction = [cos(sensing_angle), sin(sensing_angle)];
            current_position = current_position + step_size * step_direction + adjustment;
            
            % Check if the goal is visible again after a small step
            if read_sensor(goal_direction, current_position) >= distance_to_goal
                break;
            end
        end
    end
    
    % Return the updated position after following the boundary
    new_position = current_position;
end

