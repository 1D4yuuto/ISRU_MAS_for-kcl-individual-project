function path_result = astar_pathfinding_standard(start_pos, goal_pos, grid_map, robot, weights)

% Input:
%   start_pos - starting position [x, y] in grid coordinates
%   goal_pos  - goal position [x, y] in grid coordinates  
%   grid_map  - 200x200 grid map with terrain values
%   robot     - robot struct with speed and energy parameters
%   weights   - optimization weights struct
% Output:
%   path_result - struct containing path and cost information

    % Convert positions to integer grid coordinates
    start_pos = round(start_pos);
    goal_pos = round(goal_pos);
    
    % Grid dimensions
    [rows, cols] = size(grid_map);
    
    % Check bounds and validity
    if start_pos(1) < 1 || start_pos(1) > cols || start_pos(2) < 1 || start_pos(2) > rows || ...
       goal_pos(1) < 1 || goal_pos(1) > cols || goal_pos(2) < 1 || goal_pos(2) > rows
        error('Start or goal position is out of bounds');
    end
    
    % Check if start or goal is obstacle
    if grid_map(start_pos(2), start_pos(1)) == 0 || grid_map(goal_pos(2), goal_pos(1)) == 0
        error('Start or goal position is an obstacle');
    end
    
    % If start equals goal, return trivial path
    if isequal(start_pos, goal_pos)
        path_result = create_trivial_path_result(start_pos, weights);
        return;
    end
    
    % Initialize A* data structures
    open_set = [];  % [f_score, g_score, x, y]
    closed_set = false(rows, cols);
    g_score = inf(rows, cols);
    f_score = inf(rows, cols);
    came_from = zeros(rows, cols, 2);  % Parent coordinates
    
    % Starting node
    start_g = 0;
    start_h = calculate_heuristic_standard(start_pos, goal_pos, grid_map, robot, weights);
    start_f = start_g + start_h;
    
    g_score(start_pos(2), start_pos(1)) = start_g;
    f_score(start_pos(2), start_pos(1)) = start_f;
    open_set = [start_f, start_g, start_pos(1), start_pos(2)];
    
    % 8-directional movement (including diagonals)
    directions = [
        1, 0;   % East
        -1, 0;  % West  
        0, 1;   % North
        0, -1;  % South
        1, 1;   % Northeast
        -1, 1;  % Northwest
        1, -1;  % Southeast
        -1, -1  % Southwest
    ];
    
    % A* main loop
    while ~isempty(open_set)
        % Find node with lowest f_score
        [~, min_idx] = min(open_set(:, 1));
        current = open_set(min_idx, 3:4);  % [x, y]
        current_g = open_set(min_idx, 2);
        
        % Remove current from open set
        open_set(min_idx, :) = [];
        
        % Add current to closed set
        closed_set(current(2), current(1)) = true;
        
        % Check if goal reached
        if isequal(current, goal_pos)
            path = reconstruct_path(came_from, start_pos, goal_pos);
            path_result = calculate_path_costs_standard(path, grid_map, robot, weights);
            return;
        end
        
        % Explore neighbors
        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > cols || neighbor(2) < 1 || neighbor(2) > rows
                continue;
            end
            
            % Skip if obstacle or already in closed set
            if grid_map(neighbor(2), neighbor(1)) == 0 || closed_set(neighbor(2), neighbor(1))
                continue;
            end
            
            % Calculate movement cost (ignoring terrain)
            move_cost = calculate_movement_cost_standard(current, neighbor, grid_map, robot, weights);
            tentative_g = current_g + move_cost;
            
            % Check if this path is better
            if tentative_g < g_score(neighbor(2), neighbor(1))
                % Update scores
                came_from(neighbor(2), neighbor(1), :) = current;
                g_score(neighbor(2), neighbor(1)) = tentative_g;
                
                heuristic = calculate_heuristic_standard(neighbor, goal_pos, grid_map, robot, weights);
                f_score(neighbor(2), neighbor(1)) = tentative_g + heuristic;
                
                % Add to open set if not already there
                in_open = any(open_set(:, 3) == neighbor(1) & open_set(:, 4) == neighbor(2));
                if ~in_open
                    open_set(end+1, :) = [f_score(neighbor(2), neighbor(1)), ...
                                         tentative_g, neighbor(1), neighbor(2)];
                end
            end
        end
    end
    
    % No path found
    error('No path found from [%d,%d] to [%d,%d]', start_pos(1), start_pos(2), goal_pos(1), goal_pos(2));
end

%% Calculate heuristic (ignoring terrain)
function h_cost = calculate_heuristic_standard(current_pos, goal_pos, grid_map, robot, weights)
% 计算启发式成本（忽略地形因素）
    
    % Euclidean distance
    euclidean_dist = norm(current_pos - goal_pos);
    distance_meters = euclidean_dist * 50; % Convert to meters
    
    % *** 关键修改：强制terrain_factor = 1.0 ***
    terrain_factor = 1.0;  % 忽略地形，强制为1.0
    
    % Calculate weighted heuristic
    base_distance = distance_meters * terrain_factor;
    time_cost = base_distance / robot.speed;
    energy_cost = base_distance * robot.energy_move;
    
    h_cost = weights.distance * base_distance + ...
             weights.time * time_cost + ...
             weights.energy * energy_cost;
end

%% Calculate movement cost (ignoring terrain)
function cost = calculate_movement_cost_standard(current, next, grid_map, robot, weights)
% 计算移动成本（忽略地形因素）
    
    % Distance calculation
    move_vector = next - current;
    if abs(move_vector(1)) + abs(move_vector(2)) == 2  % Diagonal
        grid_distance = sqrt(2);
    else  % Orthogonal
        grid_distance = 1;
    end
    
    distance_meters = grid_distance * 50; % Convert to meters
    
    % *** 关键修改：强制terrain_multiplier = 1.0 ***
    terrain_multiplier = 1.0;  % 忽略地形，强制为1.0
    
    % Calculate costs
    actual_distance = distance_meters * terrain_multiplier;
    time_cost = actual_distance / robot.speed;
    energy_cost = actual_distance * robot.energy_move;
    
    % Weighted combination
    cost = weights.distance * actual_distance + ...
           weights.time * time_cost + ...
           weights.energy * energy_cost;
end

%% Reconstruct path from came_from matrix
function path = reconstruct_path(came_from, start_pos, goal_pos)
% Reconstruct the optimal path
    path = goal_pos;
    current = goal_pos;
    
    while ~isequal(current, start_pos)
        parent = squeeze(came_from(current(2), current(1), :))';
        path = [parent; path];
        current = parent;
    end
end

%% Calculate detailed path costs (ignoring terrain)
function path_result = calculate_path_costs_standard(path, grid_map, robot, weights)
% 计算详细路径成本（忽略地形）
    
    total_distance = 0;
    total_time = 0;
    total_energy = 0;
    
    for i = 1:size(path, 1) - 1
        current = path(i, :);
        next = path(i + 1, :);
        
        % Calculate segment cost
        move_vector = next - current;
        if abs(move_vector(1)) + abs(move_vector(2)) == 2  % Diagonal
            grid_distance = sqrt(2);
        else  % Orthogonal
            grid_distance = 1;
        end
        
        distance_meters = grid_distance * 50;
        
        % *** 关键修改：强制terrain_multiplier = 1.0 ***
        terrain_multiplier = 1.0;  % 忽略地形
        
        segment_distance = distance_meters * terrain_multiplier;
        segment_time = segment_distance / robot.speed;
        segment_energy = segment_distance * robot.energy_move;
        
        total_distance = total_distance + segment_distance;
        total_time = total_time + segment_time;
        total_energy = total_energy + segment_energy;
    end
    
    % Calculate total weighted cost
    total_cost = weights.distance * total_distance + ...
                 weights.time * total_time + ...
                 weights.energy * total_energy;
    
    % Create result struct
    path_result = struct();
    path_result.path = path;
    path_result.total_distance = total_distance;
    path_result.total_time = total_time;
    path_result.total_energy = total_energy;
    path_result.total_cost = total_cost;
    path_result.num_waypoints = size(path, 1);
end

%% Create trivial path result for same start and goal
function path_result = create_trivial_path_result(pos, weights)
% Create path result for zero-distance path
    path_result = struct();
    path_result.path = pos;
    path_result.total_distance = 0;
    path_result.total_time = 0;
    path_result.total_energy = 0;
    path_result.total_cost = 0;
    path_result.num_waypoints = 1;
end