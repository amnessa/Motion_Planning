function [x_path, y_path] = astar_tangent_bug_planner(qstart, qgoal, use_tangent_following)
    % Initialize path arrays for tracking and plotting
    x_path = [];
    y_path = [];
    
    % A* Parameters
    MOVE = 0.2;             % Movement step size
    goal_tolerance = 0.3;   % Distance threshold for reaching goal
    
    % Arena boundaries
    global arena_limits;
    xmin = arena_limits(1);
    xmax = arena_limits(2);
    ymin = arena_limits(3);
    ymax = arena_limits(4);

    % Initialize open and closed sets for A*
    open_set = [qstart];
    closed_set = [];
    
    % Cost maps
    g_score = containers.Map('KeyType', 'char', 'ValueType', 'double');  % Distance cost from start
    f_score = containers.Map('KeyType', 'char', 'ValueType', 'double');  % Total estimated cost (Path)
    came_from = containers.Map('KeyType', 'char', 'ValueType', 'any');   % Track the best path
    
    % Initialize start node costs
    start_key = pos_to_key(qstart);
    g_score(start_key) = 0;
    f_score(start_key) = heuristic(qstart, qgoal);
    
    % Movement directions for 8-connected grid
    directions = [
        1, 0; -1, 0; 0, 1; 0, -1;
        1, 1; -1, -1; -1, 1; 1, -1
    ];
    
    % A* Algorithm loop
    while ~isempty(open_set)
        % Find the node with the lowest f_score in the open set
        [current_node, current_key] = lowest_f_score(open_set, f_score);
        
        % Check if the goal is reached
        if norm(current_node - qgoal) < goal_tolerance
            [x_path, y_path] = reconstruct_path(came_from, current_node);
            return;
        end
        
        % Move current node from open set to closed set
        open_set = remove_node(open_set, current_node);
        closed_set = [closed_set; current_node];
        
        % Explore neighbors
        expanded = false;
        for i = 1:size(directions, 1)
            neighbor = current_node + MOVE * directions(i, :);
            neighbor_key = pos_to_key(neighbor);
            
            % Skip if neighbor is in closed set or blocked by an obstacle
            if node_in_set(closed_set, neighbor) || is_obstacle(current_node, directions(i, :), MOVE)
                % Check if tangent following should be activated
                if use_tangent_following
                    disp('Obstacle detected, switching to tangent boundary following.');
                    new_position = follow_boundary_tangentially(current_node, qgoal, MOVE);
                    if ~isempty(new_position)
                        current_node = new_position;
                        continue;
                    end
                end
                continue;
            end
            
            % Tentative g_score for the neighbor
            tentative_g_score = g_score(current_key) + MOVE;
            
            % Update path to neighbor if better or unvisited
            if ~isKey(g_score, neighbor_key) || tentative_g_score < g_score(neighbor_key)
                came_from(neighbor_key) = current_node;
                g_score(neighbor_key) = tentative_g_score;
                f_score(neighbor_key) = tentative_g_score + heuristic(neighbor, qgoal);
                
                % Add neighbor to open set if not already there
                if ~node_in_set(open_set, neighbor)
                    open_set = [open_set; neighbor];
                    expanded = true;
                end
            end
        end
        
        % Ensure the robot doesn't exit arena limits
        current_node = enforce_arena_limits(current_node, xmin, xmax, ymin, ymax);
        
        % Move current node to closed set if expanded
        if expanded
            closed_set = [closed_set; current_node];
        end
    end
    
    % If goal was not reached, return empty path
    x_path = [];
    y_path = [];
    disp('No path found');
end

%% Tangent Boundary Following Function (Optional)
function new_position = follow_boundary_tangentially(position, qgoal, move_step)
    global sensor_range;
    
    % Initialize parameters
    min_distance = 0.1;        % Minimum distance to maintain from obstacles
    max_distance = 0.5;        % Maximum distance to maintain from obstacles
    tangent_position = position;  % Initialize tangent position as the current position
    
    % Detect boundary edges using sensor readings
    edges = detect_boundary_edges(tangent_position, sensor_range);
    if isempty(edges)
        disp('No edges found for tangent boundary following');
        new_position = [];
        return;
    end
    
    % Pick the closest edge point
    [~, idx] = min(vecnorm(edges - tangent_position, 2, 2));
    closest_point = edges(idx, :);
    
    % Calculate angle to move tangentially along the boundary
    angle_to_obstacle = atan2(closest_point(2) - tangent_position(2), ...
                              closest_point(1) - tangent_position(1));
    
    % Adjust movement to maintain a safe distance
    distance_to_boundary = read_sensor(angle_to_obstacle, tangent_position);
    if distance_to_boundary < min_distance
        adjustment = min_distance - distance_to_boundary;  % Move outward
    elseif distance_to_boundary > max_distance
        adjustment = max_distance - distance_to_boundary;  % Move inward
    else
        adjustment = 0;  % Maintain current distance
    end
    
    % Check if the goal is visible from the current position
    goal_direction = atan2(qgoal(2) - tangent_position(2), qgoal(1) - tangent_position(1));
    distance_to_goal = norm(qgoal - tangent_position);
    if read_sensor(goal_direction, tangent_position) >= distance_to_goal
        % Clear path to the goal detected, switch back to motion-to-goal mode
        new_position = tangent_position + move_step * [cos(goal_direction), sin(goal_direction)];
        disp('Switching back to motion-to-goal');
        return;
    end
    
    % Move tangentially, maintaining safe distance from boundary
    new_position = tangent_position + move_step * [cos(angle_to_obstacle), sin(angle_to_obstacle)] ...
                   + adjustment * [cos(angle_to_obstacle), sin(angle_to_obstacle)];
end


%% Helper Functions
function k = pos_to_key(pos)
    k = sprintf('%.2f,%.2f', pos(1), pos(2));
end

function [node, key] = lowest_f_score(open_set, f_score)
    min_f = inf;
    node = [];
    key = '';
    for i = 1:size(open_set, 1)
        current_node = open_set(i, :);
        current_key = pos_to_key(current_node);
        if isKey(f_score, current_key) && f_score(current_key) < min_f
            min_f = f_score(current_key);
            node = current_node;
            key = current_key;
        end
    end
end

function h = heuristic(node, goal)
    h = norm(node - goal);
end

function [x_path, y_path] = reconstruct_path(came_from, current)
    path = [];
    path = [current];
    while isKey(came_from, pos_to_key(current))
        current = came_from(pos_to_key(current));
        path = [current; path];
    end
    x_path = path(:, 1);
    y_path = path(:, 2);
end

function found = node_in_set(set, node)
    found = false;
    for i = 1:size(set, 1)
        if all(abs(set(i, :) - node) < 1e-3)
            found = true;
            return;
        end
    end
end

function open_set = remove_node(open_set, node)
    open_set = open_set(~all(abs(open_set - node) < 1e-3, 2), :);
end

function is_blocked = is_obstacle(current_position, direction, step_size)
    angle = atan2(direction(2), direction(1));
    distance_to_obstacle = read_sensor(angle, current_position);
    is_blocked = distance_to_obstacle < step_size * 0.9;
end

function constrained_position = enforce_arena_limits(position, xmin, xmax, ymin, ymax)
    % Ensure the robot stays within the arena limits
    constrained_position = [
        max(xmin, min(xmax, position(1))), ...
        max(ymin, min(ymax, position(2)))
    ];
end


%% Helper function to decide turn direction based on heuristic
function dir = choose_turn_direction(current_angle, goal_position)
    % Assume `dir = 1` for left turn, `dir = -1` for right turn based on A* heuristic
    dir = 1;
    if heuristic([cos(current_angle), sin(current_angle)], goal_position) > heuristic([-cos(current_angle), -sin(current_angle)], goal_position)
        dir = -1;
    end
end

%% Function for calculating the tangent direction
function angRev = findTangentAngle(angle, dir)
    if dir == 1
        angleX = angle - pi/2;
    else
        angleX = angle + pi/2;
    end
    if angleX > 2*pi
        angleX = angleX - 2*pi;
    elseif angleX < 0
        angleX = angleX + 2*pi;
    end
    angRev = angleX;
end

function edges = detect_boundary_edges(position, sensor_range)
    % Detect boundary edges using sensor readings around the robot
    edges = [];  % Initialize an empty list of edges
    
    % Sweep the sensor in 360 degrees to get readings
    num_readings = 360;
    angles = linspace(0, 2 * pi, num_readings);
    edge_threshold = 0.2;  % Difference threshold to detect discontinuities
    last_reading = read_sensor(angles(1), position);
    
    for i = 2:num_readings
        % Get the distance to the closest obstacle at this angle
        current_reading = read_sensor(angles(i), position);
        
        % Check if there is a significant change in readings (discontinuity)
        if abs(current_reading - last_reading) > edge_threshold && current_reading < sensor_range
            % Record this angle as an edge
            edge_position = position + current_reading * [cos(angles(i)), sin(angles(i))];
            edges = [edges; edge_position];
        end
        
        % Update the last reading for the next iteration
        last_reading = current_reading;
    end
    
    % Ensure unique edges
    edges = unique(edges, 'rows');
end