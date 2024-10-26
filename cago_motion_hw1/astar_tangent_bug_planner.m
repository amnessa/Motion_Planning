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
        for i = 1:size(directions, 1)
            neighbor = current_node + MOVE * directions(i, :);
            neighbor_key = pos_to_key(neighbor);
            
            % Skip if neighbor is in closed set or blocked by an obstacle
            if node_in_set(closed_set, neighbor) || is_obstacle(current_node, directions(i, :), MOVE)
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
                end
            end
        end
    end
    
    % If goal was not reached, return empty path
    x_path = [];
    y_path = [];
    disp('No path found');
end

%% Enforce arena limits function
function constrained_position = enforce_arena_limits(position, xmin, xmax, ymin, ymax)
    % Ensure the robot stays within the arena limits
    constrained_position = [
        max(xmin, min(xmax, position(1))), ...
        max(ymin, min(ymax, position(2)))
    ];
end

%% Detect intersection points with boundary
function edges = detect_boundary_edges(position, sensor_range)
    angles = linspace(-pi, pi, 36);
    edges = [];
    for angle = angles
        distance = read_sensor(angle, position);
        if distance < sensor_range
            edge = position + distance * [cos(angle), sin(angle)];
            edges = [edges; edge];
        end
    end
end

%% Tangent Boundary Following Function (optional for Bug algorithm)
function new_position = follow_boundary_tangentially(position, qgoal, move_step)
    global sensor_range;
    tangent_position = position;  % Initialize tangent position as current position
    
    % Get intersection points around boundary within sensor range
    edges = detect_boundary_edges(position, sensor_range);
    if isempty(edges)
        disp('No edges found for boundary following');
        return;
    end

    % Select the edge that minimizes heuristic cost to goal
    [~, min_index] = min(vecnorm(edges - qgoal, 2, 2));
    selected_edge = edges(min_index, :);
    
    % Move in the tangent direction
    tangent_direction = atan2(selected_edge(2) - position(2), selected_edge(1) - position(1));
    new_position = position + move_step * [cos(tangent_direction), sin(tangent_direction)];
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
